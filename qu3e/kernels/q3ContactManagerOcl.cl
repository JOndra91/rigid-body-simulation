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

typedef struct
{
    q3Mat3 rotation;
    q3Vec3 position;
} q3Transform;

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

inline r32 q3Abs(r32 a)
{
    return fabs(a);
}

/**
 * Matrix matrix multiplication
 */
inline q3Mat3 mmMul(const q3Mat m, const q3Mat n) {
    return m * n;
}

/**
 * Transform transform multiplication
 */
inline q3Transform ttMul(const q3Transform t, const q3Transform u)
{
    q3Transform v;
    v.rotation = mmMul( t.rotation, u.rotation );
    v.position = mvMul( t.rotation, u.position ) + t.position;
    return v;
}

/**
 * Matrix vector multiplication
 */
inline q3Vec3 mvMul(const q3Mat3 m, const q3Vec3 v)
{
  return (q3Vec3)(
    m.ex.x * v.x + m.ey.x * v.y + m.ez.x * v.z,
    m.ex.y * v.x + m.ey.y * v.y + m.ez.y * v.z,
    m.ex.z * v.x + m.ey.z * v.y + m.ez.z * v.z
  );
}

inline const q3Mat3 mTranspose( const q3Mat3 m )
{
    return q3Mat3(
        m.ex.x, m.ey.x, m.ez.x,
        m.ex.y, m.ey.y, m.ez.y,
        m.ex.z, m.ey.z, m.ez.z
    );
}

inline q3Vec3 mvMulT( const q3Mat3 r, const q3Vec3 v )
{
    return mvMul(mTranspose( r ), v);
}

inline q3Vec3 mCol0( const q3Mat3 m )
{
    return q3Vec3( m.ex.x, m.ey.x, m.ez.x );
}

inline q3Vec3 mCol1( const q3Mat3 m )
{
    return q3Vec3( m.ex.y, m.ey.y, m.ez.y );
}

inline q3Vec3 mCol2( const q3Mat3 m )
{
    return q3Vec3( m.ex.z, m.ey.z, m.ez.z );
}

inline bool bodiesCanColide(q3Body a, q3Body b);
inline bool bodyIsAwake(q3Body body);
inline bool aabbOverlaps(q3AABB a, q3AABB b);
void solveCollision(q3ManifoldOcl *m, q3Body *bodyA, q3Box *boxA, q3Body *bodyB, q3Box *boxB);
inline bool q3TrackFaceAxis(i32 *axis, i32 n, r32 s, r32 *sMax, q3Vec3 normal, q3Vec3 *axisNormal);
inline bool q3TrackEdgeAxis(i32* axis, i32 n, r32 s, r32 *sMax, q3Vec3 normal, q3Vec3 *axisNormal);
void q3BoxToBox(q3ManifoldOcl *m, q3Body *bodyA, q3Box *boxA, q3Body *bodyB, q3Box *boxB);

kernel void testCollisions
    ( const global Indicies *indexBuffer // read only
    , global q3ContactConstraintOcl *constraintBuffer
    , global q3ManifoldOcl *manifoldBuffer
    , global q3Contact *contactBuffer
    , global q3Body *bodyBuffer
    , global q3Box *boxBuffer
    , const global aabbNode *aabbNodeBuffer // read only
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
    solveCollision(&manifold, &boxA, &boxB);

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

void solveCollision(q3ManifoldOcl *m, q3Body *bodyA, q3Box *boxA, q3Body *bodyB, q3Box *boxB) {
    m->contactCount = 0;

    q3BoxToBox(m, bodyA, boxA, bodyB, boxB);

    i32 flags = m->m_flags;
    int collisionMask = -(manifold.contactCount > 0);
    // ^ This should set mask to 0x00000000 or 0xFFFFFFFF
    m->m_flags = (
        (collisionMask & (
            (flags | ((flags & eColliding) << 1)) | eColliding
            ))
        |
        (~collisionMask & (
            ((flags & ~eWasColliding) | ((flags & eColliding) << 1))
                & ~eColliding
            ))
        );
    // ^ This is crazy mess but it's equivalent with following code:
    // if ( manifold.contactCount > 0 )
    // {
    //     if ( m_flags & eColliding )
    //         m_flags |= eWasColliding;
    //
    //     else
    //         m_flags |= eColliding;
    // }
    //
    // else
    // {
    //     if ( m_flags & eColliding )
    //     {
    //         m_flags &= ~eColliding;
    //         m_flags |= eWasColliding;
    //     }
    //
    //     else
    //         m_flags &= ~eWasColliding;
    // }
}

inline bool q3TrackFaceAxis(i32 *axis, i32 n, r32 s, r32 *sMax, q3Vec3 normal, q3Vec3 *axisNormal)
{
    if ( s > 0.0f )
        return true;

    if ( s > *sMax )
    {
        *sMax = s;
        *axis = n;
        *axisNormal = normal;
    }

    return false;
}


inline bool q3TrackEdgeAxis(i32* axis, i32 n, r32 s, r32 *sMax, q3Vec3 normal, q3Vec3 *axisNormal);
{
    if ( s > 0.0f )
        return true;

    r32 l = 1.0f / q3Length( normal );
    s *= l;

    if ( s > *sMax )
    {
        *sMax = s;
        *axis = n;
        *axisNormal = normal * l;
    }

    return false;
}

void q3BoxToBox(q3ManifoldOcl *m, q3Body *bodyA, q3Box *boxA, q3Body *bodyB, q3Box *boxB) {
    q3Transform atx = bodyA->m_tx;
    q3Transform btx = bodyB->m_tx;
    q3Transform aL = boxA->localTransform;
    q3Transform bL = boxB->localTransform;
    atx = ttMul(atx, aL);
    btx = ttMul( btx, bL);
    q3Vec3 eA = boxA->e;
    q3Vec3 eB = boxB->e;

    q3Mat3 C = mTranspose( atx.rotation ) * btx.rotation;

    q3Mat3 absC;
    bool parallel = false;
    const r32 kCosTol = 1.0e-6f;
    for ( i32 i = 0; i < 3; ++i )
    {
        for ( i32 j = 0; j < 3; ++j )
        {
            r32 val = q3Abs( C[ i ][ j ] );
            absC[ i ][ j ] = val;

            parallel = parallel || (val + kCosTol >= 1.0f);
        }
    }

    // Vector from center A to center B in A's space
    q3Vec3 t = mvMulT( atx.rotation, btx.position - atx.position );

    // Query states
    r32 s;
    r32 aMax = -Q3_R32_MAX;
    r32 bMax = -Q3_R32_MAX;
    r32 eMax = -Q3_R32_MAX;
    i32 aAxis = ~0;
    i32 bAxis = ~0;
    i32 eAxis = ~0;
    q3Vec3 nA;
    q3Vec3 nB;
    q3Vec3 nE;


    // Face axis checks

    // a's x axis
    s = q3Abs( t.x ) - (eA.x + q3Dot( mCol0(absC), eB ));
    if ( q3TrackFaceAxis( &aAxis, 0, s, &aMax, atx.rotation.ex, &nA ) )
        return;

    // a's y axis
    s = q3Abs( t.y ) - (eA.y + q3Dot( mCol1(absC), eB ));
    if ( q3TrackFaceAxis( &aAxis, 1, s, &aMax, atx.rotation.ey, &nA ) )
        return;

    // a's z axis
    s = q3Abs( t.z ) - (eA.z + q3Dot( mCol2(absC), eB ));
    if ( q3TrackFaceAxis( &aAxis, 2, s, &aMax, atx.rotation.ez, &nA ) )
        return;

    // b's x axis
    s = q3Abs( q3Dot( t, C.ex ) ) - (eB.x + q3Dot( absC.ex, eA ));
    if ( q3TrackFaceAxis( &bAxis, 3, s, &bMax, btx.rotation.ex, &nB ) )
        return;

    // b's y axis
    s = q3Abs( q3Dot( t, C.ey ) ) - (eB.y + q3Dot( absC.ey, eA ));
    if ( q3TrackFaceAxis( &bAxis, 4, s, &bMax, btx.rotation.ey, &nB ) )
        return;

    // b's z axis
    s = q3Abs( q3Dot( t, C.ez ) ) - (eB.z + q3Dot( absC.ez, eA ));
    if ( q3TrackFaceAxis( &bAxis, 5, s, &bMax, btx.rotation.ez, &nB ) )
        return;


    if ( !parallel )
    {
        // Edge axis checks
        r32 rA;
        r32 rB;

        // Cross( a.x, b.x )
        rA = eA.y * absC[ 0 ][ 2 ] + eA.z * absC[ 0 ][ 1 ];
        rB = eB.y * absC[ 2 ][ 0 ] + eB.z * absC[ 1 ][ 0 ];
        s = q3Abs( t.z * C[ 0 ][ 1 ] - t.y * C[ 0 ][ 2 ] ) - (rA + rB);
        if ( q3TrackEdgeAxis( &eAxis, 6, s, &eMax, q3Vec3( r32( 0.0 ), -C[ 0 ][ 2 ], C[ 0 ][ 1 ] ), &nE ) )
            return;

        // Cross( a.x, b.y )
        rA = eA.y * absC[ 1 ][ 2 ] + eA.z * absC[ 1 ][ 1 ];
        rB = eB.x * absC[ 2 ][ 0 ] + eB.z * absC[ 0 ][ 0 ];
        s = q3Abs( t.z * C[ 1 ][ 1 ] - t.y * C[ 1 ][ 2 ] ) - (rA + rB);
        if ( q3TrackEdgeAxis( &eAxis, 7, s, &eMax, q3Vec3( r32( 0.0 ), -C[ 1 ][ 2 ], C[ 1 ][ 1 ] ), &nE ) )
            return;

        // Cross( a.x, b.z )
        rA = eA.y * absC[ 2 ][ 2 ] + eA.z * absC[ 2 ][ 1 ];
        rB = eB.x * absC[ 1 ][ 0 ] + eB.y * absC[ 0 ][ 0 ];
        s = q3Abs( t.z * C[ 2 ][ 1 ] - t.y * C[ 2 ][ 2 ] ) - (rA + rB);
        if ( q3TrackEdgeAxis( &eAxis, 8, s, &eMax, q3Vec3( r32( 0.0 ), -C[ 2 ][ 2 ], C[ 2 ][ 1 ] ), &nE ) )
            return;

        // Cross( a.y, b.x )
        rA = eA.x * absC[ 0 ][ 2 ] + eA.z * absC[ 0 ][ 0 ];
        rB = eB.y * absC[ 2 ][ 1 ] + eB.z * absC[ 1 ][ 1 ];
        s = q3Abs( t.x * C[ 0 ][ 2 ] - t.z * C[ 0 ][ 0 ] ) - (rA + rB);
        if ( q3TrackEdgeAxis( &eAxis, 9, s, &eMax, q3Vec3( C[ 0 ][ 2 ], r32( 0.0 ), -C[ 0 ][ 0 ] ), &nE ) )
            return;

        // Cross( a.y, b.y )
        rA = eA.x * absC[ 1 ][ 2 ] + eA.z * absC[ 1 ][ 0 ];
        rB = eB.x * absC[ 2 ][ 1 ] + eB.z * absC[ 0 ][ 1 ];
        s = q3Abs( t.x * C[ 1 ][ 2 ] - t.z * C[ 1 ][ 0 ] ) - (rA + rB);
        if ( q3TrackEdgeAxis( &eAxis, 10, s, &eMax, q3Vec3( C[ 1 ][ 2 ], r32( 0.0 ), -C[ 1 ][ 0 ] ), &nE ) )
            return;

        // Cross( a.y, b.z )
        rA = eA.x * absC[ 2 ][ 2 ] + eA.z * absC[ 2 ][ 0 ];
        rB = eB.x * absC[ 1 ][ 1 ] + eB.y * absC[ 0 ][ 1 ];
        s = q3Abs( t.x * C[ 2 ][ 2 ] - t.z * C[ 2 ][ 0 ] ) - (rA + rB);
        if ( q3TrackEdgeAxis( &eAxis, 11, s, &eMax, q3Vec3( C[ 2 ][ 2 ], r32( 0.0 ), -C[ 2 ][ 0 ] ), &nE ) )
            return;

        // Cross( a.z, b.x )
        rA = eA.x * absC[ 0 ][ 1 ] + eA.y * absC[ 0 ][ 0 ];
        rB = eB.y * absC[ 2 ][ 2 ] + eB.z * absC[ 1 ][ 2 ];
        s = q3Abs( t.y * C[ 0 ][ 0 ] - t.x * C[ 0 ][ 1 ] ) - (rA + rB);
        if ( q3TrackEdgeAxis( &eAxis, 12, s, &eMax, q3Vec3( -C[ 0 ][ 1 ], C[ 0 ][ 0 ], r32( 0.0 ) ), &nE ) )
            return;

        // Cross( a.z, b.y )
        rA = eA.x * absC[ 1 ][ 1 ] + eA.y * absC[ 1 ][ 0 ];
        rB = eB.x * absC[ 2 ][ 2 ] + eB.z * absC[ 0 ][ 2 ];
        s = q3Abs( t.y * C[ 1 ][ 0 ] - t.x * C[ 1 ][ 1 ] ) - (rA + rB);
        if ( q3TrackEdgeAxis( &eAxis, 13, s, &eMax, q3Vec3( -C[ 1 ][ 1 ], C[ 1 ][ 0 ], r32( 0.0 ) ), &nE ) )
            return;

        // Cross( a.z, b.z )
        rA = eA.x * absC[ 2 ][ 1 ] + eA.y * absC[ 2 ][ 0 ];
        rB = eB.x * absC[ 1 ][ 2 ] + eB.y * absC[ 0 ][ 2 ];
        s = q3Abs( t.y * C[ 2 ][ 0 ] - t.x * C[ 2 ][ 1 ] ) - (rA + rB);
        if ( q3TrackEdgeAxis( &eAxis, 14, s, &eMax, q3Vec3( -C[ 2 ][ 1 ], C[ 2 ][ 0 ], r32( 0.0 ) ), &nE ) )
            return;
    }
}
