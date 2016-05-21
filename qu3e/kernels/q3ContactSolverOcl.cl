#include "types.cl"
#include "functions.cl"

kernel void prepare
    ( global q3Body *bodies
    , global q3VelocityStateOcl *velocities
    , r32 dt
    , q3Vec3 gravity
    , const global uint *indicies
    , uint bodyCount
    ) {

    uint global_x = (uint)get_global_id(0);

    if(global_x >= bodyCount)
    {
        return;
    }

    // if(global_x == 0) {
    //     printf("sizeof(i32) = %u\n", sizeof(i32));
    //     printf("sizeof(r32) = %u\n", sizeof(r32));
    //     printf("sizeof(u32) = %u\n", sizeof(u32));
    //     printf("sizeof(q3Vec3) = %u\n", sizeof(q3Vec3));
    //     printf("sizeof(q3Mat3) = %u\n", sizeof(q3Mat3));
    //     printf("sizeof(q3Transform) = %u\n", sizeof(q3Transform));
    //     printf("sizeof(q3VelocityStateOcl) = %u\n", sizeof(q3VelocityStateOcl));
    //     printf("sizeof(q3ContactStateOcl) = %u\n", sizeof(q3ContactStateOcl));
    //     printf("sizeof(q3ContactConstraintStateOcl) = %u\n", sizeof(q3ContactConstraintStateOcl));
    //     printf("sizeof(q3Body) = %u\n", sizeof(q3Body));
    // }

    uint idx = indicies[global_x];
    q3Body body = bodies[idx];
    q3VelocityStateOcl v;

    if (body.m_flags & eDynamic)
    {
        v = velocities[global_x];
        body_ApplyLinearForce( &body, gravity * body.m_gravityScale );

        // Calculate world space intertia tensor
        q3Mat3 r = body.m_tx.rotation;
        body.m_invInertiaWorld = mmMul(r, mmMul(body.m_invInertiaModel, mTranspose( r )));

        // Integrate velocity
        body.m_linearVelocity += (body.m_force * body.m_invMass) * dt;
        body.m_angularVelocity += mvMul(body.m_invInertiaWorld, body.m_torque) * dt;

        // From Box2D!
        // Apply damping.
        // ODE: dv/dt + c * v = 0
        // Solution: v(t) = v0 * exp(-c * t)
        // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
        // v2 = exp(-c * dt) * v1
        // Pade approximation:
        // v2 = v1 * 1 / (1 + c * dt)
        body.m_linearVelocity *= 1.0f / (1.0f + dt * 0.0f);
        body.m_angularVelocity *= 1.0f / (1.0f + dt * 0.1f);

    }

    v.v = body.m_linearVelocity;
    v.w = body.m_angularVelocity;

    bodies[idx] = body;
    velocities[global_x] = v;
}

typedef union {
    float f;
    uint u;
} mixType;

kernel void integrate
    ( global q3Body *bodies
    , const global q3VelocityStateOcl *velocities
    , const r32 dt
    , const global uint *indicies
    , const uint bodyCount
    , const uint sleepEnable
    , const r32 SLEEP_TIME
    , global mixType *minBuffer
    ) {

    uint global_x = (uint)get_global_id(0);
    bool terminated = false;
    uint idx;
    q3Body body;

    if(global_x >= bodyCount)
    {
        terminated = true;
    }
    else
    {
        idx = indicies[global_x];
        body = bodies[idx];
    }


    if (body.m_flags & eStatic)
    {
        terminated = true;
    }

    if(!terminated) {

        const q3VelocityStateOcl v = velocities[global_x];

        body.m_linearVelocity = v.v;
        body.m_angularVelocity = v.w;

        // Integrate position
        body.m_worldCenter += body.m_linearVelocity * dt;
        body.m_q = qIntegrate( body.m_q, body.m_angularVelocity, dt );
        body.m_tx.rotation = qToMat3(body.m_q);
    }

    if (sleepEnable)
    {
        // Find minimum sleep time of the entire island
        r32 minSleepTime = FLT_MAX;

        const r32 sqrLinVel = q3Dot( body.m_linearVelocity, body.m_linearVelocity );
        const r32 cbAngVel = q3Dot( body.m_angularVelocity, body.m_angularVelocity );
        const r32 linTol = Q3_SLEEP_LINEAR;
        const r32 angTol = Q3_SLEEP_ANGULAR;

        if ( sqrLinVel > linTol || cbAngVel > angTol )
        {
            minSleepTime = 0.0;
            body.m_sleepTime = 0.0;
        }
        else
        {
            body.m_sleepTime += dt;
            minSleepTime = body.m_sleepTime;
        }

        if(!terminated) {
            minBuffer[body.m_islandId].u = UINT_MAX;
        }

        const float maxSleepTime = SLEEP_TIME * 2.0;

        barrier(CLK_GLOBAL_MEM_FENCE);

        uint minSleepTimeInt = min(minSleepTime, maxSleepTime) * ((float)(UINT_MAX) / maxSleepTime);
        if(!terminated) {
            atomic_min(&minBuffer[body.m_islandId].u, minSleepTimeInt);
        }

        if(!terminated) {
            if(minBuffer[body.m_islandId].u == minSleepTimeInt) {
                minBuffer[body.m_islandId].f = minSleepTime;
            }
        }

        barrier(CLK_GLOBAL_MEM_FENCE);

        if(!terminated) {
            minSleepTime = minBuffer[body.m_islandId].f;
        }

        // Put entire island to sleep so long as the minimum found sleep time
        // is below the threshold. If the minimum sleep time reaches below the
        // sleeping threshold, the entire island will be reformed next step
        // and sleep test will be tried again.
        if ( minSleepTime > SLEEP_TIME )
        {
            body_SetToSleep(&body);
        }
    }

    if(!terminated) {
        bodies[idx] = body;
    }
}

kernel void preSolve
    ( global q3VelocityStateOcl *m_velocities
    , global q3ContactConstraintStateOcl *m_contactConstraints
    , global q3ContactStateOcl *m_contactStates
    , global uint2 *batches
    , uint batchOffset, uint batchSize, int m_enableFriction, r32 dt
    )
{
  uint global_x = (uint)get_global_id(0);

  if(global_x >= batchSize)
  {
    return;
  }

  uint totalOffset = global_x + batchOffset;

  uint2 plan = batches[totalOffset];

  global q3ContactStateOcl *c_ = m_contactStates + plan.x;
  global q3ContactConstraintStateOcl *cs = m_contactConstraints + c_->constraintIndex;

  q3VelocityStateOcl A = m_velocities[ cs->indexA ];
  q3VelocityStateOcl B = m_velocities[ cs->indexB ];

  for(uint i = 0; i < plan.y; ++i) {
    global q3ContactStateOcl *c = c_ + i;

    // Preload values
    q3Vec3 ra = c->ra;
    q3Vec3 rb = c->rb;
    q3Vec3 normal = cs->normal;
    r32 cs_mA = cs->mA;
    r32 cs_mB = cs->mB;
    q3Mat3 cs_iA = cs->iA;
    q3Mat3 cs_iB = cs->iB;
    q3Vec3 tangentVectors[2];

    tangentVectors[0] = cs->tangentVectors[0];
    tangentVectors[1] = cs->tangentVectors[1];

    // Precalculate JM^-1JT for contact and friction constraints
    q3Vec3 raCn = q3Cross( ra, normal );
    q3Vec3 rbCn = q3Cross( rb, normal );
    r32 nm = cs_mA + cs_mB;
    r32 tm[ 2 ];

    tm[ 0 ] = nm;
    tm[ 1 ] = nm;

    nm += q3Dot( raCn, mvMul(cs_iA, raCn) ) + q3Dot( rbCn, mvMul(cs_iB, rbCn) );
    c->normalMass = q3Invert( nm );

    for ( i32 i = 0; i < 2; ++i )
    {
      q3Vec3 raCt = q3Cross( tangentVectors[ i ], ra );
      q3Vec3 rbCt = q3Cross( tangentVectors[ i ], rb );
      tm[ i ] += q3Dot( raCt, mvMul(cs_iA, raCt) ) + q3Dot( rbCt, mvMul(cs_iB, rbCt) );
      c->tangentMass[ i ] = q3Invert( tm[ i ] );
    }

    // Precalculate bias factor
    c->bias = -Q3_BAUMGARTE * (1.0f / dt) * q3Min( 0.0f, c->penetration + Q3_PENETRATION_SLOP );

    // Warm start contact
    q3Vec3 P = normal * c->normalImpulse;

    if ( m_enableFriction )
    {
        P += tangentVectors[ 0 ] * c->tangentImpulse[ 0 ];
        P += tangentVectors[ 1 ] * c->tangentImpulse[ 1 ];
    }

    A.v -= P * cs_mA;
    A.w -= mvMul(cs_iA, q3Cross( ra, P ));

    B.v += P * cs_mB;
    B.w += mvMul(cs_iB, q3Cross( rb, P ));

    // Add in restitution bias
    r32 dv = q3Dot( B.v + q3Cross( B.w, rb ) - A.v - q3Cross( A.w, ra ), normal );

    if ( dv < -1.0f ) {
      c->bias += -(cs->restitution) * dv;
    }
  }

  m_velocities[ cs->indexA ] = A;
  m_velocities[ cs->indexB ] = B;
}


// constant modifier doesn't work on nVidia
kernel void solve
    ( global q3VelocityStateOcl *m_velocities
    , global q3ContactConstraintStateOcl *m_contactConstraints
    , global q3ContactStateOcl *m_contactStates
    , global uint2 *batches
    , uint batchOffset, uint batchSize, int m_enableFriction
    )
{
  uint global_x = (uint)get_global_id(0);

  if(global_x >= batchSize)
  {
    return;
  }

  uint totalOffset = global_x + batchOffset;

  uint2 plan = batches[totalOffset];

  global q3ContactStateOcl *c_ = m_contactStates + plan.x;
  global q3ContactConstraintStateOcl *cs = m_contactConstraints + c_->constraintIndex;

  q3VelocityStateOcl A = m_velocities[ cs->indexA ];
  q3VelocityStateOcl B = m_velocities[ cs->indexB ];

  for(uint i = 0; i < plan.y; ++i) {
    global q3ContactStateOcl *c = c_ + i;

    r32 c_normalImpulse = c->normalImpulse;
    r32 cs_mA = cs->mA;
    r32 cs_mB = cs->mB;

    q3Mat3 cs_iA = cs->iA;
    q3Mat3 cs_iB = cs->iB;

    q3Vec3 cs_normal = cs->normal;
    q3Vec3 c_ra = c->ra;
    q3Vec3 c_rb = c->rb;

    // relative velocity at contact
    q3Vec3 dv = B.v + q3Cross( B.w, c_rb ) - A.v - q3Cross( A.w, c_ra );

    // Friction
    if ( m_enableFriction )
    {
      for ( i32 i = 0; i < 2; ++i )
      {
        q3Vec3 cs_tangentVectorsI = cs->tangentVectors[ i ];
        r32 c_tangentImpulseI = c->tangentImpulse[ i ];

        r32 lambda = -q3Dot( dv, cs_tangentVectorsI ) * c->tangentMass[ i ];

        // Calculate frictional impulse
        r32 maxLambda = cs->friction * c_normalImpulse;

        // Clamp frictional impulse
        r32 oldPT = c_tangentImpulseI;
        c_tangentImpulseI = q3Clamp( -maxLambda, maxLambda, oldPT + lambda );
        c->tangentImpulse[ i ] = c_tangentImpulseI;
        lambda = c_tangentImpulseI - oldPT;

        // Apply friction impulse
        q3Vec3 impulse = cs_tangentVectorsI * lambda;
        A.v -= impulse * cs_mA;
        A.w -= mvMul(cs_iA, q3Cross( c_ra, impulse ));

        B.v += impulse * cs_mB;
        B.w += mvMul(cs_iB, q3Cross( c_rb, impulse ));
      }
    }

    // Normal
    {
      dv = B.v + q3Cross( B.w, c_rb ) - A.v - q3Cross( A.w, c_ra );

      // Normal impulse
      r32 vn = q3Dot( dv, cs_normal );

      // Factor in positional bias to calculate impulse scalar j
      r32 lambda = c->normalMass * (-vn + c->bias);

      // Clamp impulse
      r32 tempPN = c_normalImpulse;
      c_normalImpulse = q3Max( tempPN + lambda, 0.0f );
      c->normalImpulse = c_normalImpulse;
      lambda = c_normalImpulse - tempPN;

      // Apply impulse
      q3Vec3 impulse = cs_normal * lambda;
      A.v -= impulse * cs_mA;
      A.w -= mvMul(cs_iA, q3Cross( c_ra, impulse ));

      B.v += impulse * cs_mB;
      B.w += mvMul(cs_iB, q3Cross( c_rb, impulse ));
    }
  }

  m_velocities[ cs->indexA ] = A;
  m_velocities[ cs->indexB ] = B;
}
