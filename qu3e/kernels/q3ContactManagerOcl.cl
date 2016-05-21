typedef int    i32;
typedef uint   u32;
typedef float  r32;
typedef uchar u8;
typedef float3 q3Vec3;
typedef float4 q3Quaternion;

#define Q3_R32_MAX FLT_MAX
#define assert(cnd) do{if(!(cnd)) printf("Assert failed: %s\n", #cnd);} while(0)

typedef union
{
    struct {
        q3Vec3 ex;
        q3Vec3 ey;
        q3Vec3 ez;
    };
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
    r32 tangentImpulse[2];    // Accumulated friction impulse
    r32 bias;                    // Restitution + baumgarte
    r32 normalMass;                // Normal constraint mass
    r32 tangentMass[2];        // Tangent constraint mass
    q3FeaturePairOcl fp;            // Features on A and B for this contact
    u8 warmStarted;                // Used for debug rendering
} q3ContactOcl;

typedef struct
{
    q3Vec3 tangentVectors[2];    // Tangent vectors
    q3Vec3 normal;                // From A to B
    i32 contactCount;
    u32 sensor;
} q3ManifoldOcl;

enum constraintFlags
{
    eColliding    = 0x00000001, // Set when contact collides during a step
    eWasColliding = 0x00000002, // Set when two objects stop colliding
    // eIsland       = 0x00000004, // Not needed for openCL kernel
    eRemove       = 0x00000008,
};

typedef struct
{
    r32 friction;
    r32 restitution;
    i32 m_flags;
    i32 __padding;
} q3ContactConstraintOcl;

typedef struct
{
    q3Vec3 v;
    q3FeaturePairOcl f;
} q3ClipVertex;

typedef struct {
    r32 tangentImpulse[2];
    r32 normalImpulse;
    q3FeaturePairOcl fp;
} q3OldContactOcl;

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

inline r32 q3Abs(r32 a)
{
    return fabs(a);
}

inline q3Vec3 vAbs(q3Vec3 a)
{
    return (q3Vec3)(fabs(a.x), fabs(a.y), fabs(a.z));
}

inline r32 q3Length( const q3Vec3 v )
{
    return length(v);
}

inline r32 q3Sign(r32 a) {
    return sign(a);
}

inline const q3Vec3 q3Normalize( const q3Vec3 v )
{
    return normalize(v);
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

/*
 * Transform matrix multiplication
 */
inline q3Vec3 tvMul(const q3Transform tx, const q3Vec3 v)
{
    return mvMul(tx.rotation, v) + tx.position;
}

inline const q3Mat3 mTranspose( const q3Mat3 m )
{
    q3Mat3 n;
    n.ex = (q3Vec3)(m.ex.x, m.ey.x, m.ez.x);
    n.ey = (q3Vec3)(m.ex.y, m.ey.y, m.ez.y);
    n.ez = (q3Vec3)(m.ex.z, m.ey.z, m.ez.z);
    return n;
}

inline q3Vec3 mvMulT( const q3Mat3 r, const q3Vec3 v )
{
    return mvMul(mTranspose( r ), v);
}

inline q3Vec3 mCol0( const q3Mat3 m )
{
    return (q3Vec3)( m.ex.x, m.ey.x, m.ez.x );
}

inline q3Vec3 mCol1( const q3Mat3 m )
{
    return (q3Vec3)( m.ex.y, m.ey.y, m.ez.y );
}

inline q3Vec3 mCol2( const q3Mat3 m )
{
    return (q3Vec3)( m.ex.z, m.ey.z, m.ez.z );
}

inline void vSet(q3Vec3 *v, r32 x, r32 y, r32 z) {
    v->x = x;
    v->y = y;
    v->z = z;
}

inline void mSetRows(q3Mat3 *m, q3Vec3 x, q3Vec3 y, q3Vec3 z) {
    m->ex = x;
    m->ey = y;
    m->ez = z;
}

inline bool bodiesCanColide(const q3Body a, const q3Body b);
inline bool bodyIsAwake(const q3Body body);
inline bool aabbOverlaps(q3AABB a, q3AABB b);
void solveCollision(global q3ContactOcl *contacts, q3ContactConstraintOcl *c, q3ManifoldOcl *m, q3Body *bodyA, q3Box *boxA, q3Body *bodyB, q3Box *boxB);
inline bool q3TrackFaceAxis(i32 *axis, i32 n, r32 s, r32 *sMax, q3Vec3 normal, q3Vec3 *axisNormal);
inline bool q3TrackEdgeAxis(i32* axis, i32 n, r32 s, r32 *sMax, q3Vec3 normal, q3Vec3 *axisNormal);
void q3BoxToBox(global q3ContactOcl *contacts, q3ManifoldOcl *m, q3Body *bodyA, q3Box *boxA, q3Body *bodyB, q3Box *boxB);
void q3ComputeIncidentFace(q3Transform itx, q3Vec3 e, q3Vec3 n, q3ClipVertex* out);
void q3ComputeReferenceEdgesAndBasis(q3Vec3 eR, q3Transform rtx, q3Vec3 n, i32 axis, u8* out, q3Mat3* basis, q3Vec3* e);
void q3SupportEdge(q3Transform tx, q3Vec3 e, q3Vec3 n, q3Vec3* aOut, q3Vec3* bOut);
inline void q3EdgesContact( q3Vec3 *CA, q3Vec3 *CB, const q3Vec3 PA, const q3Vec3 QA, const q3Vec3 PB, const q3Vec3 QB);
inline void q3ComputeBasis( const q3Vec3 a, q3Vec3* b, q3Vec3* c );
i32 q3Clip(const q3Vec3 rPos, const q3Vec3 e, u8* clipEdges, const q3Mat3 basis, q3ClipVertex* incident, q3ClipVertex* outVerts, r32* outDepths);
i32 q3Orthographic( r32 sign, r32 e, i32 axis, i32 clipEdge, q3ClipVertex* in, i32 inCount, q3ClipVertex* out );

#define offsetofinstance(obj, member) (((char*)&(obj).member)-((char*)&(obj)))
#define offsetof(type, member) ({type type ## _; offsetofinstance(type ## _, member);})

kernel void testCollisions
    ( const global Indicies *indexBuffer // read only
    , global q3ContactConstraintOcl *constraintBuffer
    , global q3ManifoldOcl *manifoldBuffer
    , global q3ContactOcl *contactBuffer
    , const global q3OldContactOcl *oldContactBuffer // read only
    , const global q3Body *bodyBuffer
    , const global q3Box *boxBuffer
    , const global aabbNode *aabbNodeBuffer // read only
    , uint constraintCount
    )
{
    uint global_x = (uint)get_global_id(0);

    if(global_x >= constraintCount)
    {
        return;
    }

    // if(global_x == 0) {
    //     printf("sizeof(i32) = %u\n", sizeof(i32));
    //     printf("sizeof(r32) = %u\n", sizeof(r32));
    //     printf("sizeof(u32) = %u\n", sizeof(u32));
    //     printf("sizeof(q3Vec3) = %u\n", sizeof(q3Vec3));
    //     printf("sizeof(q3Mat3) = %u\n", sizeof(q3Mat3));
    //     printf("sizeof(Indicies) = %u\n", sizeof(Indicies));
    //     printf("sizeof(q3AABB) = %u\n", sizeof(q3AABB));
    //     printf("sizeof(aabbNode) = %u\n", sizeof(aabbNode));
    //     printf("sizeof(q3Transform) = %u\n", sizeof(q3Transform));
    //     printf("sizeof(q3Box) = %u\n", sizeof(q3Box));
    //     printf("sizeof(q3Body) = %u\n", sizeof(q3Body));
    //     printf("sizeof(q3FeaturePairOcl) = %u\n", sizeof(q3FeaturePairOcl));
    //     printf("sizeof(q3ContactOcl) = %u\n", sizeof(q3ContactOcl));
    //     printf("sizeof(q3ManifoldOcl) = %u\n", sizeof(q3ManifoldOcl));
    //     printf("sizeof(q3ContactConstraintOcl) = %u\n", sizeof(q3ContactConstraintOcl));
    //     printf("sizeof(q3ClipVertex) = %u\n", sizeof(q3ClipVertex));
    //     printf("sizeof(q3OldContactOcl) = %u\n", sizeof(q3OldContactOcl));
    //
    //     printf("offsetof(Indicies.boxA) = %u\n", offsetof(Indicies, boxA));
    //     printf("offsetof(Indicies.boxB) = %u\n", offsetof(Indicies, boxB));
    //     printf("offsetof(Indicies.bodyA) = %u\n", offsetof(Indicies, bodyA));
    //     printf("offsetof(Indicies.bodyB) = %u\n", offsetof(Indicies, bodyB));
    //
    //     printf("offsetof(q3AABB.min) = %u\n", offsetof(q3AABB, min));
    //     printf("offsetof(q3AABB.max) = %u\n", offsetof(q3AABB, max));
    //
    //     printf("offsetof(aabbNode.aabb) = %u\n", offsetof(aabbNode, aabb));
    //     printf("offsetof(aabbNode.height) = %u\n", offsetof(aabbNode, height));
    //
    //     printf("offsetof(q3ContactConstraintOcl.friction) = %u\n", offsetof(q3ContactConstraintOcl, friction));
    //     printf("offsetof(q3ContactConstraintOcl.restitution) = %u\n", offsetof(q3ContactConstraintOcl, restitution));
    //     printf("offsetof(q3ContactConstraintOcl.m_flags) = %u\n", offsetof(q3ContactConstraintOcl, m_flags));
    //
    //     printf("offsetof(q3ClipVertex.v) = %u\n", offsetof(q3ClipVertex, v));
    //     printf("offsetof(q3ClipVertex.f) = %u\n", offsetof(q3ClipVertex, f));
    //
    //     printf("offsetof(q3OldContactOcl.tangentImpulse[0]) = %u\n", offsetof(q3OldContactOcl, tangentImpulse[0]));
    //     printf("offsetof(q3OldContactOcl.tangentImpulse[1]) = %u\n", offsetof(q3OldContactOcl, tangentImpulse[1]));
    //     printf("offsetof(q3OldContactOcl.normalImpulse) = %u\n", offsetof(q3OldContactOcl, normalImpulse));
    //     printf("offsetof(q3OldContactOcl.fp) = %u\n", offsetof(q3OldContactOcl, fp));
    //
    //     printf("offsetof(q3ManifoldOcl.tangentVectors[0]) = %u\n", offsetof(q3ManifoldOcl, tangentVectors[0]));
    //     printf("offsetof(q3ManifoldOcl.tangentVectors[1]) = %u\n", offsetof(q3ManifoldOcl, tangentVectors[1]));
    //     printf("offsetof(q3ManifoldOcl.normal) = %u\n", offsetof(q3ManifoldOcl, normal));
    //     printf("offsetof(q3ManifoldOcl.contactCount) = %u\n", offsetof(q3ManifoldOcl, contactCount));
    //     printf("offsetof(q3ManifoldOcl.sensor) = %u\n", offsetof(q3ManifoldOcl, sensor));
    //
    //     printf("offsetof(q3FeaturePairOcl.key) = %u\n", offsetof(q3FeaturePairOcl, key));
    //     printf("offsetof(q3FeaturePairOcl.inR) = %u\n", offsetof(q3FeaturePairOcl, inR));
    //     printf("offsetof(q3FeaturePairOcl.outR) = %u\n", offsetof(q3FeaturePairOcl, outR));
    //     printf("offsetof(q3FeaturePairOcl.inI) = %u\n", offsetof(q3FeaturePairOcl, inI));
    //     printf("offsetof(q3FeaturePairOcl.outI) = %u\n", offsetof(q3FeaturePairOcl, outI));
    //
    //     printf("offsetof(q3ContactOcl.position) = %u\n", offsetof(q3ContactOcl, position));
    //     printf("offsetof(q3ContactOcl.penetration) = %u\n", offsetof(q3ContactOcl, penetration));
    //     printf("offsetof(q3ContactOcl.normalImpulse) = %u\n", offsetof(q3ContactOcl, normalImpulse));
    //     printf("offsetof(q3ContactOcl.tangentImpulse[0]) = %u\n", offsetof(q3ContactOcl, tangentImpulse[0]));
    //     printf("offsetof(q3ContactOcl.tangentImpulse[1]) = %u\n", offsetof(q3ContactOcl, tangentImpulse[1]));
    //     printf("offsetof(q3ContactOcl.bias) = %u\n", offsetof(q3ContactOcl, bias));
    //     printf("offsetof(q3ContactOcl.normalMass) = %u\n", offsetof(q3ContactOcl, normalMass));
    //     printf("offsetof(q3ContactOcl.tangentMass[0]) = %u\n", offsetof(q3ContactOcl, tangentMass[0]));
    //     printf("offsetof(q3ContactOcl.tangentMass[1]) = %u\n", offsetof(q3ContactOcl, tangentMass[1]));
    //     printf("offsetof(q3ContactOcl.fp) = %u\n", offsetof(q3ContactOcl, fp));
    //     printf("offsetof(q3ContactOcl.warmStarted) = %u\n", offsetof(q3ContactOcl, warmStarted));
    //
    //     printf("offsetof(q3Body.m_invInertiaModel) = %u\n", offsetof(q3Body, m_invInertiaModel));
    //     printf("offsetof(q3Body.m_invInertiaWorld) = %u\n", offsetof(q3Body, m_invInertiaWorld));
    //     printf("offsetof(q3Body.m_mass) = %u\n", offsetof(q3Body, m_mass));
    //     printf("offsetof(q3Body.m_invMass) = %u\n", offsetof(q3Body, m_invMass));
    //     printf("offsetof(q3Body.m_linearVelocity) = %u\n", offsetof(q3Body, m_linearVelocity));
    //     printf("offsetof(q3Body.m_angularVelocity) = %u\n", offsetof(q3Body, m_angularVelocity));
    //     printf("offsetof(q3Body.m_force) = %u\n", offsetof(q3Body, m_force));
    //     printf("offsetof(q3Body.m_torque) = %u\n", offsetof(q3Body, m_torque));
    //     printf("offsetof(q3Body.m_tx) = %u\n", offsetof(q3Body, m_tx));
    //     printf("offsetof(q3Body.m_q) = %u\n", offsetof(q3Body, m_q));
    //     printf("offsetof(q3Body.m_localCenter) = %u\n", offsetof(q3Body, m_localCenter));
    //     printf("offsetof(q3Body.m_worldCenter) = %u\n", offsetof(q3Body, m_worldCenter));
    //     printf("offsetof(q3Body.m_sleepTime) = %u\n", offsetof(q3Body, m_sleepTime));
    //     printf("offsetof(q3Body.m_gravityScale) = %u\n", offsetof(q3Body, m_gravityScale));
    //     printf("offsetof(q3Body.m_layers) = %u\n", offsetof(q3Body, m_layers));
    //     printf("offsetof(q3Body.m_flags) = %u\n", offsetof(q3Body, m_flags));
    //     printf("offsetof(q3Body.m_bodyIndex) = %u\n", offsetof(q3Body, m_bodyIndex));
    //     printf("offsetof(q3Body.m_islandIndex) = %u\n", offsetof(q3Body, m_islandIndex));
    //     printf("offsetof(q3Body.m_islandId) = %u\n", offsetof(q3Body, m_islandId));
    //
    //     printf("offsetof(q3Box.localTransform) = %u\n", offsetof(q3Box, localTransform));
    //     printf("offsetof(q3Box.e) = %u\n", offsetof(q3Box, e));
    //     printf("offsetof(q3Box.friction) = %u\n", offsetof(q3Box, friction));
    //     printf("offsetof(q3Box.restitution) = %u\n", offsetof(q3Box, restitution));
    //     printf("offsetof(q3Box.density) = %u\n", offsetof(q3Box, density));
    //     printf("offsetof(q3Box.broadPhaseIndex) = %u\n", offsetof(q3Box, broadPhaseIndex));
    //     printf("offsetof(q3Box.sensor) = %u\n", offsetof(q3Box, sensor));
    //     printf("offsetof(q3Box.m_boxIndex) = %u\n", offsetof(q3Box, m_boxIndex));
    //     printf("offsetof(q3Box.m_bodyIndex) = %u\n", offsetof(q3Box, m_bodyIndex));
    //
    //     printf("offsetof(q3Transform.rotation) = %u\n", offsetof(q3Transform, rotation));
    //     printf("offsetof(q3Transform.position) = %u\n", offsetof(q3Transform, position));
    // }

    const Indicies indicies = indexBuffer[global_x];
    const q3Body bodyA = bodyBuffer[indicies.bodyA];
    const q3Body bodyB = bodyBuffer[indicies.bodyB];

    if(!bodiesCanColide(bodyA, bodyB)) {
        constraintBuffer[global_x].m_flags |= eRemove;
        return;
    }

    if(!bodyIsAwake(bodyA) && !bodyIsAwake(bodyB)) {
        return;
    }

    const q3Box boxA = boxBuffer[indicies.boxA];
    const q3Box boxB = boxBuffer[indicies.boxB];
    const q3AABB aabbA = aabbNodeBuffer[boxA.broadPhaseIndex].aabb;
    const q3AABB aabbB = aabbNodeBuffer[boxB.broadPhaseIndex].aabb;

    if(!aabbOverlaps(aabbA, aabbB)) {
        constraintBuffer[global_x].m_flags |= eRemove;
        return;
    }

    q3ContactConstraintOcl constraint = constraintBuffer[global_x];
    q3ManifoldOcl manifold = manifoldBuffer[global_x];
    q3ManifoldOcl oldManifold = manifold;
    q3Vec3 ot0 = oldManifold.tangentVectors[0];
    q3Vec3 ot1 = oldManifold.tangentVectors[1];

    global q3ContactOcl *contacts = contactBuffer + (global_x * 8);

    // Solve collision
    solveCollision(contacts, &constraint, &manifold, &bodyA, &boxA, &bodyB, &boxB);
    q3ComputeBasis(manifold.normal, manifold.tangentVectors, manifold.tangentVectors + 1);

    for ( i32 i = 0; i < manifold.contactCount; ++i )
    {
        global q3ContactOcl *c = contacts + i;
        c->tangentImpulse[0] = c->tangentImpulse[1] = c->normalImpulse = 0.0f;
        u8 oldWarmStart = c->warmStarted;
        c->warmStarted = 0;

        const global q3OldContactOcl *ocPtr = oldContactBuffer + (8 * global_x);
        for ( i32 j = 0; j < oldManifold.contactCount; ++j )
        {
            const global q3OldContactOcl *oc = ocPtr + j;
            if ( c->fp.key == oc->fp.key )
            {
                c->normalImpulse = oc->normalImpulse;

                // Attempt to re-project old friction solutions
                q3Vec3 friction = ot0 * oc->tangentImpulse[0] + ot1 * oc->tangentImpulse[1];
                c->tangentImpulse[0] = q3Dot( friction, manifold.tangentVectors[0] );
                c->tangentImpulse[1] = q3Dot( friction, manifold.tangentVectors[1] );
                c->warmStarted = max(oldWarmStart, (u8)(oldWarmStart + 1));
                break;
            }
        }
    }

    constraintBuffer[global_x] = constraint;
    manifoldBuffer[global_x] = manifold;
}

inline bool bodiesCanColide( const q3Body a, const q3Body b) {
    return !(a.m_bodyIndex == b.m_bodyIndex
        || (!(a.m_flags & eDynamic) && !(b.m_flags & eDynamic))
        || !(a.m_layers & b.m_layers));
}

inline bool bodyIsAwake(const q3Body body) {
    return (body.m_flags & eAwake) != 0;
}

inline bool aabbOverlaps(q3AABB a, q3AABB b) {
    return
        !(a.max.x < b.min.x || a.min.x > b.max.x)
            &&
        !(a.max.y < b.min.y || a.min.y > b.max.y)
            &&
        !(a.max.z < b.min.z || a.min.z > b.max.z);
}

void solveCollision(global q3ContactOcl *contacts, q3ContactConstraintOcl *c, q3ManifoldOcl *m, q3Body *bodyA, q3Box *boxA, q3Body *bodyB, q3Box *boxB) {
    m->contactCount = 0;

    q3BoxToBox(contacts, m, bodyA, boxA, bodyB, boxB);

    i32 flags = c->m_flags;
    int collisionMask = -(m->contactCount > 0);
    // ^ This should set mask to 0x00000000 or 0xFFFFFFFF
    c->m_flags = (
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
    // if ( m->contactCount > 0 )
    // {
    //     if ( flags & eColliding )
    //         flags |= eWasColliding;
    //
    //     else
    //         flags |= eColliding;
    // }
    //
    // else
    // {
    //     if ( flags & eColliding )
    //     {
    //         flags &= ~eColliding;
    //         flags |= eWasColliding;
    //     }
    //
    //     else
    //         flags &= ~eWasColliding;
    // }
    //
    // c->m_flags = flags;
}

inline bool q3TrackFaceAxis(i32 *axis, i32 n, r32 s, r32 *sMax, q3Vec3 normal, q3Vec3 *axisNormal) {
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


inline bool q3TrackEdgeAxis(i32* axis, i32 n, r32 s, r32 *sMax, q3Vec3 normal, q3Vec3 *axisNormal) {
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


void q3ComputeReferenceEdgesAndBasis(q3Vec3 eR, q3Transform rtx, q3Vec3 n, i32 axis, u8* out, q3Mat3* basis, q3Vec3* e)
{
    n = mvMulT( rtx.rotation, n );

    if ( axis >= 3 )
        axis -= 3;

    switch ( axis )
    {
    case 0:
        if ( n.x > 0.0f )
        {
            out[0] = 1;
            out[1] = 8;
            out[2] = 7;
            out[3] = 9;

            vSet(e, eR.y, eR.z, eR.x);
            mSetRows(basis, rtx.rotation.ey, rtx.rotation.ez, rtx.rotation.ex);
        }

        else
        {
            out[0] = 11;
            out[1] = 3;
            out[2] = 10;
            out[3] = 5;

            vSet(e, eR.z, eR.y, eR.x);
            mSetRows(basis, rtx.rotation.ez, rtx.rotation.ey, -rtx.rotation.ex);
        }
        break;

    case 1:
        if ( n.y > 0.0f )
        {
            out[0] = 0;
            out[1] = 1;
            out[2] = 2;
            out[3] = 3;

            vSet(e, eR.z, eR.x, eR.y);
            mSetRows(basis, rtx.rotation.ez, rtx.rotation.ex, rtx.rotation.ey);
        }

        else
        {
            out[0] = 4;
            out[1] = 5;
            out[2] = 6;
            out[3] = 7;

            vSet(e, eR.z, eR.x, eR.y);
            mSetRows(basis, rtx.rotation.ez, -rtx.rotation.ex, -rtx.rotation.ey);
        }
        break;

    case 2:
        if ( n.z > 0.0f )
        {
            out[0] = 11;
            out[1] = 4;
            out[2] = 8;
            out[3] = 0;

            vSet(e, eR.y, eR.x, eR.z);
            mSetRows(basis, -rtx.rotation.ey, rtx.rotation.ex, rtx.rotation.ez);
        }

        else
        {
            out[0] = 6;
            out[1] = 10;
            out[2] = 2;
            out[3] = 9;

            vSet(e, eR.y, eR.x, eR.z);
            mSetRows(basis, -rtx.rotation.ey, -rtx.rotation.ex, -rtx.rotation.ez);
        }
        break;
    }
}

void q3ComputeIncidentFace(q3Transform itx, q3Vec3 e, q3Vec3 n, q3ClipVertex* out) {
    n = -mvMulT( itx.rotation, n );
    q3Vec3 absN = vAbs( n );

    if ( absN.x > absN.y && absN.x > absN.z )
    {
        if ( n.x > 0.0f )
        {
            vSet(&out[0].v,  e.x,  e.y, -e.z );
            vSet(&out[1].v,  e.x,  e.y,  e.z );
            vSet(&out[2].v,  e.x, -e.y,  e.z );
            vSet(&out[3].v,  e.x, -e.y, -e.z );

            out[0].f.inI = 9;
            out[0].f.outI = 1;
            out[1].f.inI = 1;
            out[1].f.outI = 8;
            out[2].f.inI = 8;
            out[2].f.outI = 7;
            out[3].f.inI = 7;
            out[3].f.outI = 9;
        }

        else
        {
            vSet(&out[0].v, -e.x, -e.y,  e.z );
            vSet(&out[1].v, -e.x,  e.y,  e.z );
            vSet(&out[2].v, -e.x,  e.y, -e.z );
            vSet(&out[3].v, -e.x, -e.y, -e.z );

            out[0].f.inI = 5;
            out[0].f.outI = 11;
            out[1].f.inI = 11;
            out[1].f.outI = 3;
            out[2].f.inI = 3;
            out[2].f.outI = 10;
            out[3].f.inI = 10;
            out[3].f.outI = 5;
        }
    }

    else if ( absN.y > absN.x && absN.y > absN.z )
    {
        if ( n.y > 0.0f )
        {
            vSet(&out[0].v, -e.x,  e.y,  e.z );
            vSet(&out[1].v,  e.x,  e.y,  e.z );
            vSet(&out[2].v,  e.x,  e.y, -e.z );
            vSet(&out[3].v, -e.x,  e.y, -e.z );

            out[0].f.inI = 3;
            out[0].f.outI = 0;
            out[1].f.inI = 0;
            out[1].f.outI = 1;
            out[2].f.inI = 1;
            out[2].f.outI = 2;
            out[3].f.inI = 2;
            out[3].f.outI = 3;
        }

        else
        {
            vSet(&out[0].v,  e.x, -e.y,  e.z );
            vSet(&out[1].v, -e.x, -e.y,  e.z );
            vSet(&out[2].v, -e.x, -e.y, -e.z );
            vSet(&out[3].v,  e.x, -e.y, -e.z );

            out[0].f.inI = 7;
            out[0].f.outI = 4;
            out[1].f.inI = 4;
            out[1].f.outI = 5;
            out[2].f.inI = 5;
            out[2].f.outI = 6;
            out[3].f.inI = 5;
            out[3].f.outI = 6;
        }
    }

    else
    {
        if ( n.z > 0.0f )
        {
            vSet(&out[0].v, -e.x,  e.y,  e.z );
            vSet(&out[1].v, -e.x, -e.y,  e.z );
            vSet(&out[2].v,  e.x, -e.y,  e.z );
            vSet(&out[3].v,  e.x,  e.y,  e.z );

            out[0].f.inI = 0;
            out[0].f.outI = 11;
            out[1].f.inI = 11;
            out[1].f.outI = 4;
            out[2].f.inI = 4;
            out[2].f.outI = 8;
            out[3].f.inI = 8;
            out[3].f.outI = 0;
        }

        else
        {
            vSet(&out[0].v,  e.x, -e.y, -e.z );
            vSet(&out[1].v, -e.x, -e.y, -e.z );
            vSet(&out[2].v, -e.x,  e.y, -e.z );
            vSet(&out[3].v,  e.x,  e.y, -e.z );

            out[0].f.inI = 9;
            out[0].f.outI = 6;
            out[1].f.inI = 6;
            out[1].f.outI = 10;
            out[2].f.inI = 10;
            out[2].f.outI = 2;
            out[3].f.inI = 2;
            out[3].f.outI = 9;
        }
    }

    for ( i32 i = 0; i < 4; ++i )
        out[i].v = tvMul( itx, out[i].v );
}

#define InFront( a ) \
    ((a) < 0.0f)

#define Behind( a ) \
    ((a) >= 0.0f)

#define On( a ) \
    ((a) < 0.005f && (a) > -0.005f)

i32 q3Orthographic( r32 _sign, r32 e, i32 axis, i32 clipEdge, q3ClipVertex* in, i32 inCount, q3ClipVertex* out )
{
    i32 outCount = 0;
    q3ClipVertex a = in[ inCount - 1 ];
    r32 v[3];

    for ( i32 i = 0; i < inCount; ++i )
    {
        q3ClipVertex b = in[i];

        v[0] = a.v.x; v[1] = a.v.y; v[2] = a.v.z;
        r32 da = _sign * v[axis] - e;

        v[0] = b.v.x; v[1] = b.v.y; v[2] = b.v.z;
        r32 db = _sign * v[axis] - e;

        q3ClipVertex cv;
        cv.f.key = ~0;

        // B
        if ( ((InFront( da ) && InFront( db )) || On( da ) || On( db )) )
        {
            assert( outCount < 8 );
            out[outCount++] = b;
        }

        // I
        else if ( InFront( da ) && Behind( db ) )
        {
            cv.f = b.f;
            cv.v = a.v + (b.v - a.v) * (da / (da - db));
            cv.f.outR = clipEdge;
            cv.f.outI = 0;
            assert( outCount < 8 );
            out[outCount++] = cv;
        }

        // I, B
        else if ( Behind( da ) && InFront( db ) )
        {
            cv.f = a.f;
            cv.v = a.v + (b.v - a.v) * (da / (da - db));
            cv.f.inR = clipEdge;
            cv.f.inI = 0;
            assert( outCount < 8 );
            out[outCount++] = cv;

            assert( outCount < 8 );
            out[outCount++] = b;
        }

        a = b;
    }

    return outCount;
}

i32 q3Clip(const q3Vec3 rPos, const q3Vec3 e, u8* clipEdges, const q3Mat3 basis, q3ClipVertex* incident, q3ClipVertex* outVerts, r32* outDepths)
{
    i32 inCount = 4;
    i32 outCount;
    q3ClipVertex in[8];
    q3ClipVertex out[8];

    for(int i = 0; i < 8; ++i) {
        in[i].f.key = ~0;
        out[i].f.key = ~0;
    }

    for ( i32 i = 0; i < 4; ++i )
        in[i].v = mvMulT( basis, incident[i].v - rPos );

    outCount = q3Orthographic( 1.0f, e.x, 0, clipEdges[0], in, inCount, out );

    if ( !outCount )
        return 0;

    inCount = q3Orthographic( 1.0f, e.y, 1, clipEdges[1], out, outCount, in );

    if ( !inCount )
        return 0;

    outCount = q3Orthographic( -1.0f, e.x, 0, clipEdges[2], in, inCount, out );

    if ( !outCount )
        return 0;

    inCount = q3Orthographic( -1.0f, e.y, 1, clipEdges[3], out, outCount, in );

    // Keep incident vertices behind the reference face
    outCount = 0;
    for ( i32 i = 0; i < inCount; ++i )
    {
        r32 d = in[i].v.z - e.z;

        if ( d <= 0.0f )
        {
            outVerts[outCount].v = mvMul( basis, in[i].v ) + rPos;
            outVerts[outCount].f = in[i].f;
            outDepths[outCount++] = d;
        }
    }

    assert( outCount <= 8 );

    return outCount;
}

inline void q3EdgesContact( q3Vec3 *CA, q3Vec3 *CB, const q3Vec3 PA, const q3Vec3 QA, const q3Vec3 PB, const q3Vec3 QB)
{
    q3Vec3 DA = QA - PA;
    q3Vec3 DB = QB - PB;
    q3Vec3 r = PA - PB;
    r32 a = q3Dot( DA, DA );
    r32 e = q3Dot( DB, DB );
    r32 f = q3Dot( DB, r );
    r32 c = q3Dot( DA, r );

    r32 b = q3Dot( DA, DB );
    r32 denom = a * e - b * b;

    r32 TA = (b * f - c * e) / denom;
    r32 TB = (b * TA + f) / e;

    *CA = PA + DA * TA;
    *CB = PB + DB * TB;
}

void q3SupportEdge(q3Transform tx, q3Vec3 e, q3Vec3 n, q3Vec3* aOut, q3Vec3* bOut) {
    n = mvMulT( tx.rotation, n );
    q3Vec3 absN = vAbs( n );
    q3Vec3 a, b;

    // x > y
    if ( absN.x > absN.y )
    {
        // x > y > z
        if ( absN.y > absN.z )
        {
            vSet(&a, e.x, e.y, e.z);
            vSet(&b, e.x, e.y, -e.z);
        }

        // x > z > y || z > x > y
        else
        {
            vSet(&a, e.x, e.y, e.z);
            vSet(&b, e.x, -e.y, e.z);
        }
    }

    // y > x
    else
    {
        // y > x > z
        if ( absN.x > absN.z )
        {
            vSet(&a, e.x, e.y, e.z);
            vSet(&b, e.x, e.y, -e.z);
        }

        // z > y > x || y > z > x
        else
        {
            vSet(&a, e.x, e.y, e.z);
            vSet(&b, -e.x, e.y, e.z);
        }
    }

    r32 signx = q3Sign( n.x );
    r32 signy = q3Sign( n.y );
    r32 signz = q3Sign( n.z );

    a.x *= signx;
    a.y *= signy;
    a.z *= signz;
    b.x *= signx;
    b.y *= signy;
    b.z *= signz;

    *aOut = tvMul( tx, a );
    *bOut = tvMul( tx, b );
}

inline void q3ComputeBasis( const q3Vec3 a, q3Vec3* b, q3Vec3* c ) {
    // Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
    // Then 3*s*s = 1, s = sqrt(1/3) = 0.57735027. This means that at least one component of a
    // unit vector must be greater or equal to 0.57735027. Can use SIMD select operation.

    if ( q3Abs( a.x ) >= 0.57735027f )
        vSet(b, a.y, -a.x, 0.0f);
    else
        vSet(b, 0.0f, a.z, -a.y);

    *b = q3Normalize( *b );
    *c = q3Cross( a, *b );
}

void q3BoxToBox(global q3ContactOcl *contacts, q3ManifoldOcl *m, q3Body *bodyA, q3Box *boxA, q3Body *bodyB, q3Box *boxB) {
    q3Transform atx = bodyA->m_tx;
    q3Transform btx = bodyB->m_tx;
    q3Transform aL = boxA->localTransform;
    q3Transform bL = boxB->localTransform;
    atx = ttMul(atx, aL);
    btx = ttMul(btx, bL);
    q3Vec3 eA = boxA->e;
    q3Vec3 eB = boxB->e;

    q3Mat3 C = mmMul(mTranspose( atx.rotation ), btx.rotation);

    q3Mat3 absC;
    bool parallel = false;
    const r32 kCosTol = 1.0e-6f;
    r32 val;

    val = q3Abs( C.ex.x );
    absC.ex.x = val;
    parallel = parallel || (val + kCosTol >= 1.0f);

    val = q3Abs( C.ex.y );
    absC.ex.y = val;
    parallel = parallel || (val + kCosTol >= 1.0f);

    val = q3Abs( C.ex.y );
    absC.ex.y = val;
    parallel = parallel || (val + kCosTol >= 1.0f);

    val = q3Abs( C.ey.x );
    absC.ey.x = val;
    parallel = parallel || (val + kCosTol >= 1.0f);

    val = q3Abs( C.ey.y );
    absC.ey.y = val;
    parallel = parallel || (val + kCosTol >= 1.0f);

    val = q3Abs( C.ey.z );
    absC.ey.z = val;
    parallel = parallel || (val + kCosTol >= 1.0f);

    val = q3Abs( C.ez.x );
    absC.ez.x = val;
    parallel = parallel || (val + kCosTol >= 1.0f);

    val = q3Abs( C.ez.y );
    absC.ez.y = val;
    parallel = parallel || (val + kCosTol >= 1.0f);

    val = q3Abs( C.ez.z );
    absC.ez.z = val;
    parallel = parallel || (val + kCosTol >= 1.0f);
    // for ( i32 i = 0; i < 3; ++i )
    // {
    //     for ( i32 j = 0; j < 3; ++j )
    //     {
    //         r32 val = q3Abs( C.m[i][j] );
    //         absC.m[i][j] = val;
    //
    //         parallel = parallel || (val + kCosTol >= 1.0f);
    //     }
    // }

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
        rA = eA.y * absC.ex.z + eA.z * absC.ex.y;
        rB = eB.y * absC.ez.x + eB.z * absC.ey.x;
        s = q3Abs( t.z * C.ex.y - t.y * C.ex.z ) - (rA + rB);
        if ( q3TrackEdgeAxis( &eAxis, 6, s, &eMax, (q3Vec3)( 0.0f, -C.ex.z, C.ex.y ), &nE ) )
            return;

        // Cross( a.x, b.y )
        rA = eA.y * absC.ey.z + eA.z * absC.ey.y;
        rB = eB.x * absC.ez.x + eB.z * absC.ex.x;
        s = q3Abs( t.z * C.ey.y - t.y * C.ey.z ) - (rA + rB);
        if ( q3TrackEdgeAxis( &eAxis, 7, s, &eMax, (q3Vec3)( 0.0f, -C.ey.z, C.ey.y ), &nE ) )
            return;

        // Cross( a.x, b.z )
        rA = eA.y * absC.ez.z + eA.z * absC.ez.y;
        rB = eB.x * absC.ey.x + eB.y * absC.ex.x;
        s = q3Abs( t.z * C.ez.y - t.y * C.ez.z ) - (rA + rB);
        if ( q3TrackEdgeAxis( &eAxis, 8, s, &eMax, (q3Vec3)( 0.0f, -C.ez.z, C.ez.y ), &nE ) )
            return;

        // Cross( a.y, b.x )
        rA = eA.x * absC.ex.z + eA.z * absC.ex.x;
        rB = eB.y * absC.ez.y + eB.z * absC.ey.y;
        s = q3Abs( t.x * C.ex.z - t.z * C.ex.x ) - (rA + rB);
        if ( q3TrackEdgeAxis( &eAxis, 9, s, &eMax, (q3Vec3)( C.ex.z, 0.0f, -C.ex.x ), &nE ) )
            return;

        // Cross( a.y, b.y )
        rA = eA.x * absC.ey.z + eA.z * absC.ey.x;
        rB = eB.x * absC.ez.y + eB.z * absC.ex.y;
        s = q3Abs( t.x * C.ey.z - t.z * C.ey.x ) - (rA + rB);
        if ( q3TrackEdgeAxis( &eAxis, 10, s, &eMax, (q3Vec3)( C.ey.z, 0.0f, -C.ey.x ), &nE ) )
            return;

        // Cross( a.y, b.z )
        rA = eA.x * absC.ez.z + eA.z * absC.ez.x;
        rB = eB.x * absC.ey.y + eB.y * absC.ex.y;
        s = q3Abs( t.x * C.ez.z - t.z * C.ez.x ) - (rA + rB);
        if ( q3TrackEdgeAxis( &eAxis, 11, s, &eMax, (q3Vec3)( C.ez.z, 0.0f, -C.ez.x ), &nE ) )
            return;

        // Cross( a.z, b.x )
        rA = eA.x * absC.ex.y + eA.y * absC.ex.x;
        rB = eB.y * absC.ez.z + eB.z * absC.ey.z;
        s = q3Abs( t.y * C.ex.x - t.x * C.ex.y ) - (rA + rB);
        if ( q3TrackEdgeAxis( &eAxis, 12, s, &eMax, (q3Vec3)( -C.ex.y, C.ex.x, 0.0f ), &nE ) )
            return;

        // Cross( a.z, b.y )
        rA = eA.x * absC.ey.y + eA.y * absC.ey.x;
        rB = eB.x * absC.ez.z + eB.z * absC.ex.z;
        s = q3Abs( t.y * C.ey.x - t.x * C.ey.y ) - (rA + rB);
        if ( q3TrackEdgeAxis( &eAxis, 13, s, &eMax, (q3Vec3)( -C.ey.y, C.ey.x, 0.0f ), &nE ) )
            return;

        // Cross( a.z, b.z )
        rA = eA.x * absC.ez.y + eA.y * absC.ez.x;
        rB = eB.x * absC.ey.z + eB.y * absC.ex.z;
        s = q3Abs( t.y * C.ez.x - t.x * C.ez.y ) - (rA + rB);
        if ( q3TrackEdgeAxis( &eAxis, 14, s, &eMax, (q3Vec3)( -C.ez.y, C.ez.x, 0.0f ), &nE ) )
            return;
    }

    // Artificial axis bias to improve frame coherence
    const r32 kRelTol = 0.95;
    const r32 kAbsTol = 0.01;
    i32 axis;
    r32 sMax;
    q3Vec3 n;
    r32 faceMax = max( aMax, bMax );
    if ( kRelTol * eMax > faceMax + kAbsTol )
    {
        axis = eAxis;
        sMax = eMax;
        n = nE;
    }
    else
    {
        if ( kRelTol * bMax > aMax + kAbsTol )
        {
            axis = bAxis;
            sMax = bMax;
            n = nB;
        }

        else
        {
            axis = aAxis;
            sMax = aMax;
            n = nA;
        }
    }

    if ( q3Dot( n, btx.position - atx.position ) < 0.0 )
        n = -n;

    assert( axis != ~0 );

    if ( axis < 6 )
    {
        q3Transform rtx;
        q3Transform itx;
        q3Vec3 eR;
        q3Vec3 eI;
        bool flip;

        if ( axis < 3 )
        {
            rtx = atx;
            itx = btx;
            eR = eA;
            eI = eB;
            flip = false;
        }

        else
        {
            rtx = btx;
            itx = atx;
            eR = eB;
            eI = eA;
            flip = true;
            n = -n;
        }

        // Compute reference and incident edge information necessary for clipping
        q3ClipVertex incident[4];
        for(int i = 0; i < 4; ++i) {
            incident[i].f.key = ~0;
        }
        q3ComputeIncidentFace( itx, eI, n, incident );
        u8 clipEdges[4];
        q3Mat3 basis;
        q3Vec3 e;
        q3ComputeReferenceEdgesAndBasis( eR, rtx, n, axis, clipEdges, &basis, &e );

        // Clip the incident face against the reference face side planes
        q3ClipVertex out[8];
        for(int i = 0; i < 8; ++i) {
            out[i].f.key = ~0;
        }

        r32 depths[8];
        i32 outNum;
        outNum = q3Clip(rtx.position, e, clipEdges, basis, incident, out, depths);

        if ( outNum )
        {
            m->contactCount = outNum;
            m->normal = flip ? -n : n;

            for ( i32 i = 0; i < outNum; ++i )
            {
                global q3ContactOcl* c = contacts + i;

                q3FeaturePairOcl pair = out[i].f;

                if ( flip )
                {
                    u8 swap;

                    swap = pair.inI;
                    pair.inI = pair.inR;
                    pair.inR = swap;

                    swap = pair.outI;
                    pair.outI = pair.outR;
                    pair.outR = swap;
                }

                c->fp = out[i].f;
                c->position = out[i].v;
                c->penetration = depths[i];
            }
        }
    }

    else
    {
        n = mvMul(atx.rotation, n);

        if ( q3Dot( n, btx.position - atx.position ) < 0.0f )
            n = -n;

        q3Vec3 PA, QA;
        q3Vec3 PB, QB;
        q3SupportEdge( atx, eA, n, &PA, &QA );
        q3SupportEdge( btx, eB, -n, &PB, &QB );

        q3Vec3 CA, CB;
        q3EdgesContact( &CA, &CB, PA, QA, PB, QB );

        m->normal = n;
        m->contactCount = 1;

        global q3ContactOcl* c = contacts;
        q3FeaturePairOcl pair;
        pair.key = axis;
        c->fp = pair;
        c->penetration = sMax;
        c->position = (CA + CB) * 0.5f;
    }
}
