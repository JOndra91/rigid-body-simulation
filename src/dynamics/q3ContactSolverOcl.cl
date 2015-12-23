typedef struct
{
  float3 ex;
  float3 ey;
  float3 ez;
} float3x3;

typedef struct
{
  float3 w;
  float3 v;
} q3VelocityState;

typedef struct
{
  float3 ra;          // Vector from C.O.M to contact position
  float3 rb;          // Vector from C.O.M to contact position
  float penetration;      // Depth of penetration from collision
  float normalImpulse;      // Accumulated normal impulse
  float tangentImpulse[ 2 ];  // Accumulated friction impulse
  float bias;          // Restitution + baumgarte
  float normalMass;        // Normal constraint mass
  float tangentMass[ 2 ];    // Tangent constraint mass
} q3ContactState;

typedef q3ContactState q3ContactStateOcl;

typedef struct
{
  float3 center;
  float3x3 i;                   // Inverse inertia tensor
  float3 m;                      // Inverse mass
} q3BodyInfoOcl;

typedef struct
{
  float3 tangentVectors[ 2 ];  // Tangent vectors
  float3 normal;              // From A to B
  float restitution;
  float friction;
} q3ContactConstraintStateOcl;

typedef struct
{
  uint contactStateIndex;
  uint contactConstraintStateIndex;
  uint vIndex;                 // Index to velocity and bodyInfo arrays
  // It's possible to determine according to index (starting with 0: even - A, odd - B)
  //i32 isA;                   // Whether it's A or B, so we can use correct normal
} q3ContactInfoOcl;

float3 q3Cross(float3 a, float3 b)
{
  return cross(a, b);
}

float q3Dot(float3 a, float3 b)
{
  return dot(a, b);
}

float q3Clamp(float minval, float maxval, float x)
{
  return clamp(x, minval, maxval);
}

float q3Max(float a, float b)
{
  return max(a, b);
}

float3 mvMul(float3x3 m, float3 v)
{
  return (float3)(
    m.ex.x * v.x + m.ey.x * v.y + m.ez.x * v.z,
    m.ex.y * v.x + m.ey.y * v.y + m.ez.y * v.z,
    m.ex.z * v.x + m.ey.z * v.y + m.ez.z * v.z
  );
}

// TODO: Matrix multiplications
kernel void solve(constant q3BodyInfoOcl *bodyInfo, global q3VelocityState *velocityState,
    constant q3ContactInfoOcl *contactInfo, global q3ContactStateOcl *contactStates,
    constant q3ContactConstraintStateOcl *contactConstraintState, constant uint *batches,
    uint batchOffset, uint batchSize, int frictionEnabled)
{
  uint global_x = (uint)get_global_id(0);
  uint local_x = (uint)get_local_id(0);
  uint size_x = (uint)get_global_size(0);

  if(global_x > batchSize)
  {
    return;
  }

  uint totalOffset = global_x + batchOffset;

  uint idx = batches[totalOffset];


  bool isA = ((idx % 2) == 0);
  int offsetA = -(idx % 2);
  int offsetB = ((idx + 1) % 2);
  float modSign = ((float)isA) * -2.0 + 1.0; // -1.0 for A, 1.0 for B

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
  float3 dv = velB.v + q3Cross( velB.w, c.rb ) - velA.v - q3Cross( velA.w, c.ra );

  // Friction
  if ( frictionEnabled )
  {
    for ( int i = 0; i < 2; ++i )
    {
      float lambda = -q3Dot( dv, cs.tangentVectors[ i ] ) * c.tangentMass[ i ];

      // Calculate frictional impulse
      float maxLambda = cs.friction * c.normalImpulse;

      // Clamp frictional impulse
      float oldPT = c.tangentImpulse[ i ];
      c.tangentImpulse[ i ] = q3Clamp( -maxLambda, maxLambda, oldPT + lambda );
      lambda = c.tangentImpulse[ i ] - oldPT;

      // Apply friction impulse
      float3 impulse = cs.tangentVectors[ i ] * lambda;

      velA.v -= impulse * bodyA.m;
      velA.w -= mvMul(bodyA.i, q3Cross( c.ra, impulse ));

      velB.v += impulse * bodyB.m;
      velB.w += mvMul(bodyB.i, q3Cross( c.rb, impulse ));
    }
  }

  // Normal
  {
    dv = velB.v + q3Cross( velB.w, c.rb ) - velA.v - q3Cross( velA.w, c.ra );

    // Normal impulse
    float vn = q3Dot( dv, cs.normal );

    // Factor in positional bias to calculate impulse scalar j
    float lambda = c.normalMass * (-vn + c.bias);

    // Clamp impulse
    float tempPN = c.normalImpulse;
    c.normalImpulse = q3Max( tempPN + lambda, 0.0f );
    lambda = c.normalImpulse - tempPN;

    // Apply impulse
    float3 impulse = cs.normal * lambda;
    velA.v -= impulse * bodyA.m;
    velA.w -= mvMul(bodyA.i, q3Cross( c.ra, impulse ));

    velB.v += impulse * bodyB.m;
    velB.w += mvMul(bodyB.i, q3Cross( c.rb, impulse ));
  }

  // NOTE: It's possible to compute both bodies of contact at once,
  // but it would require change in batch planning.
  if(!isA)
  {
    velA = velB;
  }

  velocityState[info.vIndex] = velA;
  contactStates[csi] = c;

}
