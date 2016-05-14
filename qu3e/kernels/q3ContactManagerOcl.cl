typedef int    i32;
typedef uint   u32;
typedef float  r32;
typedef float3 q3Vec3;

typedef struct
{
  q3Vec3 ex;
  q3Vec3 ey;
  q3Vec3 ez;
} q3Mat3;

typedef struct
{
    q3Vec3 min;
    q3Vec3 max;
} q3AABB;

typedef struct
{
    // bool IsLeaf( void ) const
    // {
    //     // The right leaf does not use the same memory as the userdata,
    //     // and will always be Null (no children)
    //     return right == Null;
    // }

    // Fat AABB for leafs, bounding AABB for branches
    q3AABB aabb;

    union
    {
        i32 parent;
        i32 next; // free list
    };

    // Child indices
    struct
    {
        i32 left;
        i32 right;
    };

    // leaf = 0, free nodes = -1
    i32 height;

    // static const i32 Null = -1;
} aabbNode;

typedef struct q3Box
{
    q3Transform localTransform;
    q3Vec3 e;

    r32 friction;
    r32 restitution;
    r32 density;
    i32 broadPhaseIndex;
    u32 sensor;
    u32 m_boxIndex;
    u32 m_bodyIndex;
} q3Box;

typedef struct
{
    // enum
    // {
    //     eAwake        = 0x001,
    //     eActive       = 0x002,
    //     eAllowSleep   = 0x004,
    //     eIsland       = 0x010,
    //     eStatic       = 0x020,
    //     eDynamic      = 0x040,
    //     eKinematic    = 0x080,
    //     eLockAxisX    = 0x100,
    //     eLockAxisY    = 0x200,
    //     eLockAxisZ    = 0x400,
    // };

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
} q3Body;

typedef union
{
    struct
    {
        u8 inR;
        u8 outR;
        u8 inI;
        u8 outI;
    };

    i32 key;
} q3FeaturePairOcl;

typedef struct
{
    q3Vec3 position;            // World coordinate of contact
    r32 penetration;            // Depth of penetration from collision
    r32 normalImpulse;            // Accumulated normal impulse
    r32 tangentImpulse[ 2 ];    // Accumulated friction impulse
    r32 bias;                    // Restitution + baumgarte
    r32 normalMass;                // Normal constraint mass
    r32 tangentMass[ 2 ];        // Tangent constraint mass
    q3FeaturePairOcl fp;            // Features on A and B for this contact
    u8 warmStarted;                // Used for debug rendering
} q3ContactOcl;

typedef struct
{
    u32 A, B; // Box indicies

    q3ContactOcl contacts[ 8 ];
    q3Vec3 tangentVectors[ 2 ];    // Tangent vectors
    q3Vec3 normal;                // From A to B
    i32 contactCount;
    u32 sensor;

} q3ManifoldOcl;

typedef struct
{

    // q3ManifoldOcl manifold; -- Use separate buffer buffer
    u32 A, B; // Box indicies
    u32 bodyA, bodyB; // Body indicies

    r32 friction;
    r32 restitution;

    i32 m_flags;

    // enum
    // {
    //     eColliding    = 0x00000001, // Set when contact collides during a step
    //     eWasColliding = 0x00000002, // Set when two objects stop colliding
    //     eIsland       = 0x00000004, // For internal marking during island forming
    // };
} q3ContactConstraintOcl;

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

kernel void testCollisions
    ( global q3ContactConstraintOcl *constraintBuffer
    , global q3ManifoldOcl *manifoldBuffer
    , uint constraintCount
    )
{
  uint global_x = (uint)get_global_id(0);

  if(global_x >= constraintCount)
  {
    return;
  }

  uint totalOffset = global_x + batchOffset;

  // if(totalOffset == 0) {
  //   printf("sizeof(i32) = %u\n", sizeof(i32));
  //   printf("sizeof(r32) = %u\n", sizeof(r32));
  //   printf("sizeof(u32) = %u\n", sizeof(u32));
  //   printf("sizeof(q3Vec3) = %u\n", sizeof(q3Vec3));
  //   printf("sizeof(q3Mat3) = %u\n", sizeof(q3Mat3));
  //   printf("sizeof(q3VelocityStateOcl) = %u\n", sizeof(q3VelocityStateOcl));
  //   printf("sizeof(q3ContactStateOcl) = %u\n", sizeof(q3ContactStateOcl));
  //   printf("sizeof(q3ContactConstraintStateOcl) = %u\n", sizeof(q3ContactConstraintStateOcl));
  // }

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
