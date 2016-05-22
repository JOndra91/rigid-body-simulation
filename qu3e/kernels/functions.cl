inline q3Vec3 q3Cross(q3Vec3 a, q3Vec3 b) {
  return cross(a, b);
}

inline r32 q3Dot(q3Vec3 a, q3Vec3 b) {
  return dot(a, b);
}

inline r32 q3Clamp(r32 minval, r32 maxval, r32 x) {
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

inline r32 q3Abs(r32 a) {
    return fabs(a);
}

inline q3Vec3 vAbs(q3Vec3 a) {
    return (q3Vec3)(fabs(a.x), fabs(a.y), fabs(a.z));
}

inline q3Mat3 mAbs(q3Mat3 m) {
    q3Mat3 o;
    o.ex = vAbs(m.ex);
    o.ey = vAbs(m.ey);
    o.ez = vAbs(m.ez);

    return o;
}

inline r32 q3Length( const q3Vec3 v ) {
    return length(v);
}

inline r32 q3Sign(r32 a) {
    return sign(a);
}

inline const q3Vec3 q3Normalize( const q3Vec3 v ) {
    return normalize(v);
}

inline q3Vec3 mCol0( const q3Mat3 m ) {
    return (q3Vec3)( m.ex.x, m.ey.x, m.ez.x );
}

inline q3Vec3 mCol1( const q3Mat3 m ) {
    return (q3Vec3)( m.ex.y, m.ey.y, m.ez.y );
}

inline q3Vec3 mCol2( const q3Mat3 m ) {
    return (q3Vec3)( m.ex.z, m.ey.z, m.ez.z );
}

/**
 * Matrix vector multiplication
 */
inline q3Vec3 mvMul(const q3Mat3 m, const q3Vec3 v) {
  return (q3Vec3)(
    dot(mCol0(m), v),
    dot(mCol1(m), v),
    dot(mCol2(m), v)
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
inline q3Transform ttMul(const q3Transform t, const q3Transform u) {
    q3Transform v;
    v.rotation = mmMul( t.rotation, u.rotation );
    v.position = mvMul( t.rotation, u.position ) + t.position;
    return v;
}

/*
 * Transform matrix multiplication
 */
inline q3Vec3 tvMul(const q3Transform tx, const q3Vec3 v) {
    return mvMul(tx.rotation, v) + tx.position;
}

inline const q3Mat3 mTranspose( const q3Mat3 m ) {
    q3Mat3 n;
    n.ex = (q3Vec3)(m.ex.x, m.ey.x, m.ez.x);
    n.ey = (q3Vec3)(m.ex.y, m.ey.y, m.ez.y);
    n.ez = (q3Vec3)(m.ex.z, m.ey.z, m.ez.z);
    return n;
}

inline q3Vec3 mvMulT( const q3Mat3 r, const q3Vec3 v ) {
    return mvMul(mTranspose( r ), v);
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


void body_SetToAwake( q3Body *body ) {
    if( !(body->m_flags & eAwake) )
    {
        body->m_flags |= eAwake;
        body->m_sleepTime = 0.0f;
    }
}

void body_SetToSleep( q3Body *body ) {
    body->m_flags &= ~eAwake;
    body->m_sleepTime = 0.0f;

    // Set identities
    body->m_linearVelocity = (q3Vec3)(0.0, 0.0, 0.0);
    body->m_angularVelocity = (q3Vec3)(0.0, 0.0, 0.0);
    body->m_force = (q3Vec3)(0.0, 0.0, 0.0);
    body->m_torque = (q3Vec3)(0.0, 0.0, 0.0);
}

void body_ApplyLinearForce( q3Body *body, const q3Vec3 force ) {
    body->m_force += force * body->m_mass;

    body_SetToAwake(body);
}

inline const q3Quaternion qNormalize( const q3Quaternion q ) {
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

q3Quaternion qIntegrate( q3Quaternion q, const q3Vec3 dv, r32 dt ) {
    q3Quaternion p = (q3Quaternion)( dv.x * dt, dv.y * dt, dv.z * dt, 0.0f);

    q.x += p.x * 0.5;
    q.y += p.y * 0.5;
    q.z += p.z * 0.5;
    q.w += p.w * 0.5;

    return qNormalize( q );
}

q3Mat3 qToMat3( const q3Quaternion q ) {
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
