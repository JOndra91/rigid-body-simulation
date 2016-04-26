typedef int    i32;
typedef float  r32;
typedef float3 q3Vec3;

#define Q3_BAUMGARTE 0.2f

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
} q3VelocityState;

typedef struct
{
  q3Vec3 ra;          // Vector from C.O.M to contact position
  q3Vec3 rb;          // Vector from C.O.M to contact position
  r32 penetration;      // Depth of penetration from collision
  r32 normalImpulse;      // Accumulated normal impulse
  r32 tangentImpulse[ 2 ];  // Accumulated friction impulse
  r32 bias;          // Restitution + baumgarte
  r32 normalMass;        // Normal constraint mass
  r32 tangentMass[ 2 ];    // Tangent constraint mass
  i32 constraintIndex;
} q3ContactState;

typedef struct
{
  q3Vec3 tangentVectors[ 2 ];  // Tangent vectors
  q3Vec3 normal;        // From A to B
  q3Vec3 centerA;
  q3Vec3 centerB;
  q3Mat3 iA;
  q3Mat3 iB;
  i32 contactCount;
  r32 mA;
  r32 mB;
  r32 restitution;
  r32 friction;
  i32 indexA;
  i32 indexB;
} q3ContactConstraintState;

q3Vec3 q3Cross(q3Vec3 a, q3Vec3 b)
{
  return cross(a, b);
}

r32 q3Dot(q3Vec3 a, q3Vec3 b)
{
  return dot(a, b);
}

r32 q3Clamp(r32 minval, r32 maxval, r32 x)
{
  return clamp(x, minval, maxval);
}

r32 q3Max(r32 a, r32 b)
{
  return max(a, b);
}

r32 q3Min(r32 a, r32 b)
{
  return min(a, b);
}

q3Vec3 mvMul(q3Mat3 m, q3Vec3 v)
{
  return (q3Vec3)(
    m.ex.x * v.x + m.ey.x * v.y + m.ez.x * v.z,
    m.ex.y * v.x + m.ey.y * v.y + m.ez.y * v.z,
    m.ex.z * v.x + m.ey.z * v.y + m.ez.z * v.z
  );
}

kernel void preSolve
    ( global q3ContactConstraintState *m_contactConstraints
    , global q3ContactState m_contactStates*
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

  // if(totalOffset == 0) {
  //   printf("sizeof(i32) = %u\n", sizeof(i32));
  //   printf("sizeof(r32) = %u\n", sizeof(r32));
  //   printf("sizeof(q3Vec3) = %u\n", sizeof(q3Vec3));
  //   printf("sizeof(q3Mat3) = %u\n", sizeof(q3Mat3));
  //   printf("sizeof(q3VelocityState) = %u\n", sizeof(q3VelocityState));
  //   printf("sizeof(q3ContactState) = %u\n", sizeof(q3ContactState));
  //   printf("sizeof(q3ContactConstraintState) = %u\n", sizeof(q3ContactConstraintState));
  //   printf("sizeof(q3ContactPlan) = %u\n", sizeof(q3ContactPlan));
  // }
  //
  // return;

  global q3ContactState *c = m_contactStates + batches + totalOffset;
  global q3ContactConstraintState *cs = m_contacts + c->constraintIndex;

  q3Vec3 vA = m_velocities[ cs->indexA ].v;
  q3Vec3 wA = m_velocities[ cs->indexA ].w;
  q3Vec3 vB = m_velocities[ cs->indexB ].v;
  q3Vec3 wB = m_velocities[ cs->indexB ].w;

  // Precalculate JM^-1JT for contact and friction constraints
  q3Vec3 raCn = q3Cross( c->ra, cs->normal );
  q3Vec3 rbCn = q3Cross( c->rb, cs->normal );
  r32 nm = cs->mA + cs->mB;
  r32 tm[ 2 ];
  tm[ 0 ] = nm;
  tm[ 1 ] = nm;

  nm += q3Dot( raCn, cs->iA * raCn ) + q3Dot( rbCn, cs->iB * rbCn );
  c->normalMass = q3Invert( nm );

  for ( i32 i = 0; i < 2; ++i )
  {
    q3Vec3 raCt = q3Cross( cs->tangentVectors[ i ], c->ra );
    q3Vec3 rbCt = q3Cross( cs->tangentVectors[ i ], c->rb );
    tm[ i ] += q3Dot( raCt, cs->iA * raCt ) + q3Dot( rbCt, cs->iB * rbCt );
    c->tangentMass[ i ] = q3Invert( tm[ i ] );
  }

  // Precalculate bias factor
  c->bias = -Q3_BAUMGARTE * (1.0f / dt) * q3Min( 0.0f, c->penetration + Q3_PENETRATION_SLOP );

  // Warm start contact
  q3Vec3 P = cs->normal * c->normalImpulse;

  if ( m_enableFriction )
  {
      P += cs->tangentVectors[ 0 ] * c->tangentImpulse[ 0 ];
      P += cs->tangentVectors[ 1 ] * c->tangentImpulse[ 1 ];
  }

  vA -= P * cs->mA;
  wA -= cs->iA * q3Cross( c->ra, P );

  vB += P * cs->mB;
  wB += cs->iB * q3Cross( c->rb, P );

  // Add in restitution bias
  r32 dv = q3Dot( vB + q3Cross( wB, c->rb ) - vA - q3Cross( wA, c->ra ), cs->normal );

  if ( dv < -1.0f ) {
    c->bias += -(cs->restitution) * dv;
  }

  m_velocities[ cs->indexA ].v = vA;
  m_velocities[ cs->indexA ].w = wA;
  m_velocities[ cs->indexB ].v = vB;
  m_velocities[ cs->indexB ].w = wB;
}


// constant modifier doesn't work on nVidia
kernel void solve
    ( global q3VelocityState *m_velocities
    , global q3ContactConstraintState *m_contactConstraints
    , global q3ContactState m_contactStates*
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

  global q3ContactState *c = m_contactStates + batches + totalOffset;
  global q3ContactConstraintState *cs = m_contacts + c->constraintIndex;

  q3Vec3 vA = m_velocities[ cs->indexA ].v;
  q3Vec3 wA = m_velocities[ cs->indexA ].w;
  q3Vec3 vB = m_velocities[ cs->indexB ].v;
  q3Vec3 wB = m_velocities[ cs->indexB ].w;

  r32 c_normalImpulse = c->normalImpulse;
  r32 cs_mA = cs->mA;
  r32 cs_mB = cs->mB;

  q3Mat3 cs_iA = cs->iA;
  q3Mat3 cs_iB = cs->iB;

  q3Vec3 cs_normal = cs->normal;
  q3Vec3 c_ra = c->ra;
  q3Vec3 c_rb = c->rb;

  // relative velocity at contact
  q3Vec3 dv = vB + q3Cross( wB, c_rb ) - vA - q3Cross( wA, c_ra );

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
      vA -= impulse * cs_mA;
      wA -= mvMul(cs_iA, q3Cross( c_ra, impulse ));

      vB += impulse * cs_mB;
      wB += mvMul(cs_iB, q3Cross( c_rb, impulse ));
    }
  }

  // Normal
  {
    dv = vB + q3Cross( wB, c_rb ) - vA - q3Cross( wA, c_ra );

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
    vA -= impulse * cs_mA;
    wA -= mvMul(cs_iA, q3Cross( c_ra, impulse ));

    vB += impulse * cs_mB;
    wB += mvMul(cs_iB, q3Cross( c_rb, impulse ));
  }

  m_velocities[ cs->indexA ].v = vA;
  m_velocities[ cs->indexA ].w = wA;
  m_velocities[ cs->indexB ].v = vB;
  m_velocities[ cs->indexB ].w = wB;
}
