typedef int    i32;
typedef uint   u32;
typedef float  r32;
typedef float3 q3Vec3;
typedef float4 q3Quaternion;

#define Q3_BAUMGARTE 0.2f
#define Q3_PENETRATION_SLOP 0.05f

#define Q3_PI 3.14159265
#define Q3_SLEEP_LINEAR 0.01
#define Q3_SLEEP_ANGULAR ((3.0 / 180.0) * Q3_PI)

typedef struct
{
  q3Vec3 ex;
  q3Vec3 ey;
  q3Vec3 ez;
} q3Mat3;

typedef struct
{
  q3Vec3 w;
  q3Vec3 v;
} q3VelocityStateOcl;

typedef struct
{
  q3Vec3 ra;          // Vector from C.O.M to contact position
  q3Vec3 rb;          // Vector from C.O.M to contact position
  r32 tangentImpulse[ 2 ];  // Accumulated friction impulse
  r32 tangentMass[ 2 ];    // Tangent constraint mass
  r32 penetration;      // Depth of penetration from collision
  r32 normalImpulse;      // Accumulated normal impulse
  r32 bias;          // Restitution + baumgarte
  r32 normalMass;        // Normal constraint mass
  u32 constraintIndex;
} q3ContactStateOcl;

typedef struct
{
  q3Mat3 iA;  // inertia of body A
  q3Mat3 iB;  // inertia of body B
  q3Vec3 tangentVectors[ 2 ];    // Tangent vectors
  q3Vec3 normal;                // From A to B
  q3Vec3 centerA;
  q3Vec3 centerB;
  i32 contactCount;
  r32 mA; // mass of body A
  r32 mB; // mass of body B
  r32 restitution;
  r32 friction;
  i32 indexA;
  i32 indexB;
} q3ContactConstraintStateOcl;

enum bodyFlags
{
    eAwake        = 0x001,
    eActive       = 0x002,
    eAllowSleep   = 0x004,
    // eIsland       = 0x010,
    eStatic       = 0x020,
    eDynamic      = 0x040,
    eKinematic    = 0x080,
    eLockAxisX    = 0x100,
    eLockAxisY    = 0x200,
    eLockAxisZ    = 0x400,
};

typedef struct
{
    q3Mat3 rotation;
    q3Vec3 position;
} q3Transform;

typedef struct
{
    q3Mat3 m_invInertiaModel;
    q3Mat3 m_invInertiaWorld;
    r32 m_mass;
    r32 m_invMass;
    q3Vec3 m_linearVelocity;
    q3Vec3 m_angularVelocity;
    q3Vec3 m_force;
    q3Vec3 m_torque;
    q3Transform m_tx;
    q3Quaternion m_q;
    q3Vec3 m_localCenter;
    q3Vec3 m_worldCenter;
    r32 m_sleepTime;
    r32 m_gravityScale;
    i32 m_layers;
    i32 m_flags;
    u32 m_bodyIndex;
    i32 m_islandIndex;
    u32 m_islandId;
} q3Body;

inline q3Vec3 q3Cross(q3Vec3 a, q3Vec3 b)
{
  return cross(a, b);
}

inline r32 q3Dot(q3Vec3 a, q3Vec3 b)
{
  return dot(a, b);
}

inline r32 q3Clamp(r32 minval, r32 maxval, r32 x)
{
  return clamp(x, minval, maxval);
}

inline r32 q3Max(r32 a, r32 b)
{
  return max(a, b);
}

inline r32 q3Min(r32 a, r32 b)
{
  return min(a, b);
}

inline r32 q3Invert( r32 a )
{
  return a != 0.0f ? 1.0f / a : 0.0f;
}

/**
 * Matrix vector multiplication
 */
inline q3Vec3 mvMul(q3Mat3 m, q3Vec3 v)
{
  return (q3Vec3)(
    m.ex.x * v.x + m.ey.x * v.y + m.ez.x * v.z,
    m.ex.y * v.x + m.ey.y * v.y + m.ez.y * v.z,
    m.ex.z * v.x + m.ey.z * v.y + m.ez.z * v.z
  );
}

/**
 * Matrix matrix multiplication
 */
inline q3Mat3 mmMul(const q3Mat3 m, const q3Mat3 n) {
    q3Mat3 o;
    o.ex = mvMul(m, n.ex);
    o.ey = mvMul(m, n.ey);
    o.ez = mvMul(m, n.ez);

    return o;
}

inline const q3Mat3 mTranspose( const q3Mat3 m )
{
    q3Mat3 n;
    n.ex = (q3Vec3)(m.ex.x, m.ey.x, m.ez.x);
    n.ey = (q3Vec3)(m.ex.y, m.ey.y, m.ez.y);
    n.ez = (q3Vec3)(m.ex.z, m.ey.z, m.ez.z);
    return n;
}

void body_SetToAwake( q3Body *body )
{
    if( !(body->m_flags & eAwake) )
    {
        body->m_flags |= eAwake;
        body->m_sleepTime = 0.0f;
    }
}

void body_SetToSleep( q3Body *body )
{
    body->m_flags &= ~eAwake;
    body->m_sleepTime = 0.0f;

    // Set identities
    body->m_linearVelocity = (q3Vec3)(0.0, 0.0, 0.0);
    body->m_angularVelocity = (q3Vec3)(0.0, 0.0, 0.0);
    body->m_force = (q3Vec3)(0.0, 0.0, 0.0);
    body->m_torque = (q3Vec3)(0.0, 0.0, 0.0);
}

void body_ApplyLinearForce( q3Body *body, const q3Vec3 force )
{
    body->m_force += force * body->m_mass;

    body_SetToAwake(body);
}

inline const q3Quaternion qNormalize( const q3Quaternion q )
{
    r32 x = q.x;
    r32 y = q.y;
    r32 z = q.z;
    r32 w = q.w;

    r32 d = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

    if( d == 0 )
        w = 1.0;

    d = 1.0 / sqrt( d );

    if ( d > 1.0e-8 )
    {
        x *= d;
        y *= d;
        z *= d;
        w *= d;
    }

    return (q3Quaternion)( x, y, z, w );
}

q3Quaternion qIntegrate( q3Quaternion q, const q3Vec3 dv, r32 dt )
{
    q3Quaternion p = (q3Quaternion)( dv.x * dt, dv.y * dt, dv.z * dt, 0.0f);

    q.x += p.x * 0.5;
    q.y += p.y * 0.5;
    q.z += p.z * 0.5;
    q.w += p.w * 0.5;

    return qNormalize( q );
}

q3Mat3 qToMat3( const q3Quaternion q )
{
    r32 qx2 = q.x + q.x;
    r32 qy2 = q.y + q.y;
    r32 qz2 = q.z + q.z;
    r32 qxqx2 = q.x * qx2;
    r32 qxqy2 = q.x * qy2;
    r32 qxqz2 = q.x * qz2;
    r32 qxqw2 = q.w * qx2;
    r32 qyqy2 = q.y * qy2;
    r32 qyqz2 = q.y * qz2;
    r32 qyqw2 = q.w * qy2;
    r32 qzqz2 = q.z * qz2;
    r32 qzqw2 = q.w * qz2;

    q3Mat3 m;

    m.ex = (q3Vec3)( 1.0f - qyqy2 - qzqz2, qxqy2 + qzqw2, qxqz2 - qyqw2 );
    m.ey = (q3Vec3)( qxqy2 - qzqw2, 1.0f - qxqx2 - qzqz2, qyqz2 + qxqw2 );
    m.ez = (q3Vec3)( qxqz2 + qyqw2, qyqz2 - qxqw2, 1.0f - qxqx2 - qyqy2 );

    return m;
}

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

    if(global_x >= bodyCount)
    {
        return;
    }

    uint idx = indicies[global_x];
    q3Body body = bodies[idx];

    if (body.m_flags & eStatic)
    {
        return;
    }

    const q3VelocityStateOcl v = velocities[global_x];

    body.m_linearVelocity = v.v;
    body.m_angularVelocity = v.w;

    // Integrate position
    body.m_worldCenter += body.m_linearVelocity * dt;
    body.m_q = qIntegrate( body.m_q, body.m_angularVelocity, dt );
    body.m_tx.rotation = qToMat3(body.m_q);

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

        minBuffer[body.m_islandId].u = UINT_MAX;

        barrier(CLK_GLOBAL_MEM_FENCE);

        uint minSleepTimeInt = minSleepTime * ((float)(UINT_MAX) / 1000.0f);
        atomic_min(&minBuffer[body.m_islandId].u, minSleepTimeInt);

        if(minBuffer[body.m_islandId].u == minSleepTimeInt) {
            minBuffer[body.m_islandId].f = minSleepTime;
        }

        barrier(CLK_GLOBAL_MEM_FENCE);

        minSleepTime = minBuffer[body.m_islandId].f;

        // Put entire island to sleep so long as the minimum found sleep time
        // is below the threshold. If the minimum sleep time reaches below the
        // sleeping threshold, the entire island will be reformed next step
        // and sleep test will be tried again.
        if ( minSleepTime > SLEEP_TIME )
        {
            body_SetToSleep(&body);
        }
    }

    bodies[idx] = body;
}

kernel void preSolve
    ( global q3VelocityStateOcl *m_velocities
    , global q3ContactConstraintStateOcl *m_contactConstraints
    , global q3ContactStateOcl *m_contactStates
    , global uint *batches
    , uint batchOffset, uint batchSize, int m_enableFriction, r32 dt
    )
{
  uint global_x = (uint)get_global_id(0);

  if(global_x >= batchSize)
  {
    return;
  }

  uint totalOffset = global_x + batchOffset;

  global q3ContactStateOcl *c = m_contactStates + batches[totalOffset];
  global q3ContactConstraintStateOcl *cs = m_contactConstraints + c->constraintIndex;

  q3VelocityStateOcl A = m_velocities[ cs->indexA ];
  q3VelocityStateOcl B = m_velocities[ cs->indexB ];

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

  m_velocities[ cs->indexA ] = A;
  m_velocities[ cs->indexB ] = B;
}


// constant modifier doesn't work on nVidia
kernel void solve
    ( global q3VelocityStateOcl *m_velocities
    , global q3ContactConstraintStateOcl *m_contactConstraints
    , global q3ContactStateOcl *m_contactStates
    , global uint *batches
    , uint batchOffset, uint batchSize, int m_enableFriction
    )
{
  uint global_x = (uint)get_global_id(0);

  if(global_x >= batchSize)
  {
    return;
  }

  uint totalOffset = global_x + batchOffset;

  global q3ContactStateOcl *c = m_contactStates + batches[totalOffset];
  global q3ContactConstraintStateOcl *cs = m_contactConstraints + c->constraintIndex;

  q3VelocityStateOcl A = m_velocities[ cs->indexA ];
  q3VelocityStateOcl B = m_velocities[ cs->indexB ];

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

  m_velocities[ cs->indexA ] = A;
  m_velocities[ cs->indexB ] = B;
}
