typedef int    i32;
typedef uint   u32;
typedef float  r32;
typedef float3 q3Vec3;
typedef float4 q3Quaternion;
typedef uchar u8;

#define Q3_R32_MAX FLT_MAX
#define assert(cnd) do{if(!(cnd)) printf("Assert failed: %s\n", #cnd);} while(0)
// #define assert(cnd)

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
