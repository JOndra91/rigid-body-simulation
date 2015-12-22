typedef int i32;
typedef float r32;
typedef float3 q3Vec3;

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
} q3ContactState;

typedef q3ContactState q3ContactStateOcl;

typedef struct
{
  q3Vec3 center;
  q3Mat3 i;                   // Inverse inertia tensor
  r32 m;                      // Inverse mass
} q3BodyInfoOcl;

typedef struct
{
  q3Vec3 tangentVectors[ 2 ];  // Tangent vectors
  q3Vec3 normal;              // From A to B
  r32 restitution;
  r32 friction;
} q3ContactConstraintStateOcl;

typedef struct
{
  i32 contactStateIndex;
  i32 contactConstraintStateIndex;
  i32 vIndex;                 // Index to velocity and bodyInfo arrays
  // It's possible to determine according to index (starting with 0: even - A, odd - B)
  //i32 isA;                   // Whether it's A or B, so we can use correct normal
} q3ContactInfoOcl;

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

// TODO: Matrix multiplications
kernel void solve(global q3BodyInfoOcl *bodyInfo, global q3VelocityState *velocityState,
    global q3ContactInfoOcl *contactInfo, global q3ContactStateOcl *contactStates,
    global q3ContactConstraintStateOcl *contactConstraintState, global int *batches,
    int batchOffset, int batchSize, int frictionEnabled)
{
  int global_x = (int)get_global_id(0);
  int local_x = (int)get_local_id(0);
  int size_x = (int)get_local_size(0);

  if(global_x > batchSize)
  {
    return;
  }

  int idx = batches[global_x + batchOffset];

  bool isA = ((idx % 2) == 0);
  int offsetA = -(idx % 2);
  int offsetB = ((idx + 1) % 2);
  r32 modSign = ((r32)isA) * -2.0 + 1.0; // -1.0 for A, 1.0 for B

  q3ContactInfoOcl info = contactInfo[idx];
  int csi = contactInfo[idx].contactStateIndex;
  int ccsi = contactInfo[idx].contactConstraintStateIndex;

  int vIndexA = contactInfo[idx+offsetA].vIndex;
  int vIndexB = contactInfo[idx+offsetB].vIndex;

  q3VelocityState velA = velocityState[vIndexA];
  q3VelocityState velB = velocityState[vIndexB];
  q3BodyInfoOcl bodyA = bodyInfo[vIndexA];
  q3BodyInfoOcl bodyB = bodyInfo[vIndexB];
  q3ContactStateOcl c = contactStates[csi];
  q3ContactConstraintStateOcl cs = contactConstraintState[ccsi];

  // relative velocity at contact
  q3Vec3 dv = velB.v + q3Cross( velB.w, c.rb ) - velA.v - q3Cross( velA.w, c.ra );

  // Friction
  if ( frictionEnabled )
  {
    for ( int i = 0; i < 2; ++i )
    {
      r32 lambda = -q3Dot( dv, cs.tangentVectors[ i ] ) * c.tangentMass[ i ];

      // Calculate frictional impulse
      r32 maxLambda = cs.friction * c.normalImpulse;

      // Clamp frictional impulse
      r32 oldPT = c.tangentImpulse[ i ];
      c.tangentImpulse[ i ] = q3Clamp( -maxLambda, maxLambda, oldPT + lambda );
      lambda = c.tangentImpulse[ i ] - oldPT;

      // Apply friction impulse
      q3Vec3 impulse = cs.tangentVectors[ i ] * lambda;

      velA.v -= impulse * bodyA.m;
      velA.w -= bodyA.i * q3Cross( c.ra, impulse );

      velB.v += impulse * bodyB.m;
      velB.w += bodyB.i * q3Cross( c.rb, impulse );
    }
  }

  // Normal
  {
    dv = velB.v + q3Cross( velB.w, c.rb ) - velA.v - q3Cross( velA.w, c.ra );

    // Normal impulse
    r32 vn = q3Dot( dv, cs.normal );

    // Factor in positional bias to calculate impulse scalar j
    r32 lambda = c.normalMass * (-vn + c.bias);

    // Clamp impulse
    r32 tempPN = c.normalImpulse;
    c.normalImpulse = q3Max( tempPN + lambda, 0.0f );
    lambda = c.normalImpulse - tempPN;

    // Apply impulse
    q3Vec3 impulse = cs.normal * lambda;
    velA.v -= impulse * bodyA.m;
    velA.w -= bodyA.i * q3Cross( c.ra, impulse );

    velB.v += impulse * bodyB.m;
    velB.w += bodyB.i * q3Cross( c.rb, impulse );
  }

  // NOTE: It's possible to compute both bodies of contact at once,
  // but it would require change in batch planning.
  if(!isA)
  {
    velA = velB;
  }

  velocityState[info.vIndex] = velA;
}
