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
    u32 boxA;
    u32 boxB;
    u32 bodyA;
    u32 bodyB;
} Indicies;

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

enum bodyFlags
{
    eAwake        = 0x001,
    eActive       = 0x002,
    eAllowSleep   = 0x004,
    eIsland       = 0x010,
    eStatic       = 0x020,
    eDynamic      = 0x040,
    eKinematic    = 0x080,
    eLockAxisX    = 0x100,
    eLockAxisY    = 0x200,
    eLockAxisZ    = 0x400,
};

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
    q3ContactOcl contacts[ 8 ];
    q3Vec3 tangentVectors[ 2 ];    // Tangent vectors
    q3Vec3 normal;                // From A to B
    i32 contactCount;
    u32 sensor;
} q3ManifoldOcl;

enum constraintFlags
{
    eColliding    = 0x00000001, // Set when contact collides during a step
    eWasColliding = 0x00000002, // Set when two objects stop colliding
    eIsland       = 0x00000004, // For internal marking during island forming
    eRemove       = 0x00000008,
};

typedef struct
{
    r32 friction;
    r32 restitution;
    i32 m_flags;
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

inline bool bodiesCanColide(q3Body a, q3Body b);
inline bool bodyIsAwake(q3Body body);
inline bool aabbOverlaps(q3AABB a, q3AABB b);

kernel void testCollisions
    ( const global Indicies *indexBuffer
    , global q3ContactConstraintOcl *constraintBuffer
    , global q3ManifoldOcl *manifoldBuffer
    , global q3Body *bodyBuffer
    , global q3Box *boxBuffer
    , const global aabbNode *aabbNodeBuffer
    , uint constraintCount
    )
{
  uint global_x = (uint)get_global_id(0);

  if(global_x >= constraintCount)
  {
    return;
  }

  Indicies indicies = indexBuffer[global_x];
  q3Body bodyA = bodyBuffer[indicies.bodyA];
  q3Body bodyB = bodyBuffer[indicies.bodyB];

  if(!bodiesCanColide(bodyA, bodyB)) {
      constraintBuffer[global_x].m_flags |= eRemove;
      return;
  }

  if(!bodyIsAwake(bodyA) && !bodyIsAwake(bodyB)) {
      return;
  }

  q3Box boxA = boxBuffer[indicies.boxA];
  q3Box boxB = boxBuffer[indicies.boxB];
  q3AABB aabbA = aabbNodeBuffer[boxA.broadPhaseIndex].aabb;
  q3AABB aabbB = aabbNodeBuffer[boxB.broadPhaseIndex].aabb;

  if(!aabbOverlaps(aabbA, aabbB)) {
      constraintBuffer[global_x].m_flags |= eRemove;
      return;
  }

  q3ContactConstraintOcl constraint = constraintBuffer[global_x];
  q3ManifoldOcl manifold = manifoldBuffer[global_x];
  q3ManifoldOcl oldManifold = manifold;
  q3Vec3 ot0 = oldManifold.tangentVectors[ 0 ];
  q3Vec3 ot1 = oldManifold.tangentVectors[ 1 ];

  // Solve collision

}

inline bool bodiesCanColide( const q3Body *a, const q3Body *b) {
    return !(a->m_bodyIndex == b->m_bodyIndex
        || (!(a->m_flags & eDynamic) && !(b->m_flags & eDynamic))
        || !(a->m_layers & b->m_layers));
}

inline bool bodyIsAwake(const q3Body *body) {
    return (body->m_flags & eAwake) != 0;
}

inline bool aabbOverlaps(q3AABB a, q3AABB b) {
    return
        !(a.max.x < b.min.x || a.min.x > b.max.x)
            &&
        !(a.max.y < b.min.y || a.min.y > b.max.y)
            &&
        !(a.max.z < b.min.z || a.min.z > b.max.z);
}
