//--------------------------------------------------------------------------------------------------
/**
@file    q3Body.cpp

@author    Randy Gaul
@date    10/10/2014

    Copyright (c) 2014 Randy Gaul http://www.randygaul.net

    This software is provided 'as-is', without any express or implied
    warranty. In no event will the authors be held liable for any damages
    arising from the use of this software.

    Permission is granted to anyone to use this software for any purpose,
    including commercial applications, and to alter it and redistribute it
    freely, subject to the following restrictions:
      1. The origin of this software must not be misrepresented; you must not
         claim that you wrote the original software. If you use this software
         in a product, an acknowledgment in the product documentation would be
         appreciated but is not required.
      2. Altered source versions must be plainly marked as such, and must not
         be misrepresented as being the original software.
      3. This notice may not be removed or altered from any source distribution.
*/
//--------------------------------------------------------------------------------------------------

#include "q3Body.h"
#include "../scene/q3Scene.h"
#include "../scene/q3Container.h"
#include "q3Contact.h"
#include "../broadphase/q3BroadPhase.h"
#include "../collision/q3Box.h"

//--------------------------------------------------------------------------------------------------
// q3Body
//--------------------------------------------------------------------------------------------------
q3Body::q3Body( const q3BodyDef& def)
{
    m_linearVelocity = def.linearVelocity;
    m_angularVelocity = def.angularVelocity;
    q3Identity( m_force );
    q3Identity( m_torque );
    m_q.Set( q3Normalize( def.axis ), def.angle );
    m_tx.rotation = m_q.ToMat3( );
    m_tx.position = def.position;
    m_sleepTime = r32( 0.0 );
    m_gravityScale = def.gravityScale;
    m_layers = def.layers;
    // m_userData = def.userData;
    // m_scene = scene;
    m_flags = 0;

    if ( def.bodyType == eDynamicBody )
        m_flags |= q3Body::eDynamic;

    else
    {
        if ( def.bodyType == eStaticBody )
        {
            m_flags |= q3Body::eStatic;
            q3Identity( m_linearVelocity );
            q3Identity( m_angularVelocity );
            q3Identity( m_force );
            q3Identity( m_torque );
        }

        else if ( def.bodyType == eKinematicBody )
            m_flags |= q3Body::eKinematic;
    }

    if ( def.allowSleep )
        m_flags |= q3Body::eAllowSleep;

    if ( def.awake )
        m_flags |= q3Body::eAwake;

    if ( def.active )
        m_flags |= q3Body::eActive;

    if ( def.lockAxisX )
        m_flags |= q3Body::eLockAxisX;

    if ( def.lockAxisY )
        m_flags |= q3Body::eLockAxisY;

    if ( def.lockAxisZ )
        m_flags |= q3Body::eLockAxisZ;

}

//--------------------------------------------------------------------------------------------------
void q3BodyRef::setContainerIndex(u32 index) {
    q3Body *body = &m_container->m_bodies[index];
    body->m_containerIndex = index;
    m_bodyIndex = index;

    for(auto &box : *m_boxes) {
        box.setBodyIndex(index);
    }
}

//--------------------------------------------------------------------------------------------------
const q3BoxRef* q3BodyRef::AddBox( const q3BoxDef& def )
{
    q3Box *box;
    q3BoxRef *ref;
    u32 index = m_container->m_boxes.size();
    m_container->m_boxes.push_back(q3Box());
    box = &m_container->m_boxes.back();

    m_boxes->push_back(q3BoxRef(m_container));
    ref = &m_boxes->back();
    ref->setContainerIndex(index);
    ref->setBodyIndex(body()->m_containerIndex);
    m_container->m_boxPtrs.push_back(ref);

    q3AABB aabb;
    box->m_containerIndex = index;
    box->m_bodyIndex = body()->m_containerIndex;
    box->local = def.m_tx;
    box->e = def.m_e;
    box->ComputeAABB( body()->m_tx, &aabb );

    box->friction = def.m_friction;
    box->restitution = def.m_restitution;
    box->density = def.m_density;
    box->sensor = def.m_sensor;

    CalculateMassData( );

    m_scene->m_contactManager.m_broadphase.InsertBox( ref, aabb );
    m_scene->m_newBox = true;

    return ref;
}

//--------------------------------------------------------------------------------------------------
void q3BodyRef::RemoveBox( q3BoxRef &box_ )
{
    q3BoxRef *box = &box_;
    m_container->remove( *this, box_);

    // Remove all contacts associated with this shape
    q3ContactEdge* edge = m_contactList;
    while ( edge )
    {
        q3ContactConstraint* contact = edge->constraint;
        edge = edge->next;

        q3BoxRef* A = contact->A;
        q3BoxRef* B = contact->B;

        if ( box == A || box == B )
            m_scene->m_contactManager.RemoveContact( contact );
    }

    m_scene->m_contactManager.m_broadphase.RemoveBox( box );

    CalculateMassData( );

    m_scene->m_heap.Free( (void*)box );
}

//--------------------------------------------------------------------------------------------------
void q3BodyRef::RemoveAllBoxes( )
{
    for(auto &box : *m_boxes) {
        m_scene->m_contactManager.m_broadphase.RemoveBox( &box );
        m_container->remove(*this, box);
    }
    m_boxes->clear();

    m_scene->m_contactManager.RemoveContactsFromBody( this );
}

//--------------------------------------------------------------------------------------------------
void q3Body::ApplyLinearForce( const q3Vec3& force )
{
    m_force += force * m_mass;

    SetToAwake( );
}

//--------------------------------------------------------------------------------------------------
void q3Body::ApplyForceAtWorldPoint( const q3Vec3& force, const q3Vec3& point )
{
    m_force += force * m_mass;
    m_torque += q3Cross( point - m_worldCenter, force );

    SetToAwake( );
}

//--------------------------------------------------------------------------------------------------
void q3Body::ApplyTorque( const q3Vec3& torque )
{
    m_torque += torque;
}

//--------------------------------------------------------------------------------------------------
void q3Body::SetToAwake( )
{
    if( !(m_flags & q3Body::eAwake) )
    {
        m_flags |= q3Body::eAwake;
        m_sleepTime = r32( 0.0 );
    }
}

//--------------------------------------------------------------------------------------------------
void q3Body::SetToSleep( )
{
    m_flags &= ~q3Body::eAwake;
    m_sleepTime = r32( 0.0 );
    q3Identity( m_linearVelocity );
    q3Identity( m_angularVelocity );
    q3Identity( m_force );
    q3Identity( m_torque );
}

//--------------------------------------------------------------------------------------------------
bool q3Body::IsAwake( ) const
{
    return m_flags & q3Body::eAwake ? true : false;
}

//--------------------------------------------------------------------------------------------------
r32 q3Body::GetGravityScale( ) const
{
    return m_gravityScale;
}

//--------------------------------------------------------------------------------------------------
void q3Body::SetGravityScale( r32 scale )
{
    m_gravityScale = scale;
}

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Body::GetLocalPoint( const q3Vec3& p ) const
{
    return q3MulT( m_tx, p );
}

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Body::GetLocalVector( const q3Vec3& v ) const
{
    return q3MulT( m_tx.rotation, v );
}

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Body::GetWorldPoint( const q3Vec3& p ) const
{
    return q3Mul( m_tx, p );
}

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Body::GetWorldVector( const q3Vec3& v ) const
{
    return q3Mul( m_tx.rotation, v );
}

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Body::GetLinearVelocity( ) const
{
    return m_linearVelocity;
}


//--------------------------------------------------------------------------------------------------
void q3Body::SetLinearVelocity( const q3Vec3& v )
{
    // Velocity of static bodies cannot be adjusted
    if ( m_flags & q3Body::eStatic )
        assert( false );

    if ( q3Dot( v, v ) > r32( 0.0 ) )
    {
        SetToAwake( );
    }

    m_linearVelocity = v;
}

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Body::GetAngularVelocity( ) const
{
    return m_angularVelocity;
}

//--------------------------------------------------------------------------------------------------
void q3Body::SetAngularVelocity( const q3Vec3 v )
{
    // Velocity of static bodies cannot be adjusted
    if ( m_flags & q3Body::eStatic )
        assert( false );

    if ( q3Dot( v, v ) > r32( 0.0 ) )
    {
        SetToAwake( );
    }

    m_angularVelocity = v;
}

//--------------------------------------------------------------------------------------------------
bool q3Body::CanCollide( const q3Body *other ) const
{
    if ( this == other )
        return false;

    // Every collision must have at least one dynamic body involved
    if ( !(m_flags & q3Body::eDynamic) && !(other->m_flags & q3Body::eDynamic) )
        return false;

    if ( !(m_layers & other->m_layers) )
        return false;

    return true;
}

//--------------------------------------------------------------------------------------------------
const q3Transform q3Body::GetTransform( ) const
{
    return m_tx;
}

//--------------------------------------------------------------------------------------------------
void q3BodyRef::SetTransform( const q3Vec3& position )
{
    body()->m_worldCenter = position;

    SynchronizeProxies( );
}

//--------------------------------------------------------------------------------------------------
void q3BodyRef::SetTransform( const q3Vec3& position, const q3Vec3& axis, r32 angle )
{
    body()->m_worldCenter = position;
    body()->m_q.Set( axis, angle );
    body()->m_tx.rotation = body()->m_q.ToMat3( );

    SynchronizeProxies( );
}

//--------------------------------------------------------------------------------------------------
i32 q3Body::GetFlags( ) const
{
    return m_flags;
}

//--------------------------------------------------------------------------------------------------
void q3Body::SetLayers( i32 layers )
{
    m_layers = layers;
}

//--------------------------------------------------------------------------------------------------
i32 q3Body::GetLayers( ) const
{
    return m_layers;
}

//--------------------------------------------------------------------------------------------------
const q3Quaternion q3Body::GetQuaternion( ) const
{
    return m_q;
}

//--------------------------------------------------------------------------------------------------
void q3BodyRef::Render( q3Render* render ) const
{
    q3Transform tx = body()->m_tx;
    bool awake = body()->IsAwake( );

    for (auto box : boxes())
    {
        box.Render( tx, awake, render );
    }
}

//--------------------------------------------------------------------------------------------------
void q3BodyRef::Dump( FILE* file, i32 index ) const
{
    q3Body b = *body();

    fprintf( file, "{\n" );
    fprintf( file, "\tq3BodyDef bd;\n" );

    switch ( b.m_flags & (q3Body::eStatic | q3Body::eDynamic | q3Body::eKinematic) )
    {
    case q3Body::eStatic:
        fprintf( file, "\tbd.bodyType = q3BodyType( %d );\n", eStaticBody );
        break;

    case q3Body::eDynamic:
        fprintf( file, "\tbd.bodyType = q3BodyType( %d );\n", eDynamicBody );
        break;

    case q3Body::eKinematic:
        fprintf( file, "\tbd.bodyType = q3BodyType( %d );\n", eKinematicBody );
        break;
    }

    fprintf( file, "\tbd.position.Set( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", b.m_tx.position.x, b.m_tx.position.y, b.m_tx.position.z );
    q3Vec3 axis;
    r32 angle;
    b.m_q.ToAxisAngle( &axis, &angle );
    fprintf( file, "\tbd.axis.Set( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", axis.x, axis.y, axis.z );
    fprintf( file, "\tbd.angle = r32( %.15lf );\n", angle );
    fprintf( file, "\tbd.linearVelocity.Set( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", b.m_linearVelocity.x, b.m_linearVelocity.y, b.m_linearVelocity.z );
    fprintf( file, "\tbd.angularVelocity.Set( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", b.m_angularVelocity.x, b.m_angularVelocity.y, b.m_angularVelocity.z );
    fprintf( file, "\tbd.gravityScale = r32( %.15lf );\n", b.m_gravityScale );
    fprintf( file, "\tbd.layers = %d;\n", b.m_layers );
    fprintf( file, "\tbd.allowSleep = bool( %d );\n", b.m_flags & q3Body::eAllowSleep );
    fprintf( file, "\tbd.awake = bool( %d );\n", b.m_flags & q3Body::eAwake );
    fprintf( file, "\tbd.awake = bool( %d );\n", b.m_flags & q3Body::eAwake );
    fprintf( file, "\tbd.lockAxisX = bool( %d );\n", b.m_flags & q3Body::eLockAxisX );
    fprintf( file, "\tbd.lockAxisY = bool( %d );\n", b.m_flags & q3Body::eLockAxisY );
    fprintf( file, "\tbd.lockAxisZ = bool( %d );\n", b.m_flags & q3Body::eLockAxisZ );
    fprintf( file, "\tbodies[ %d ] = scene.CreateBody( bd );\n\n", index );

    for(auto &boxRef : boxes())
    {
        q3Box *box = boxRef.box();
        fprintf( file, "\t{\n" );
        fprintf( file, "\t\tq3BoxDef sd;\n" );
        fprintf( file, "\t\tsd.SetFriction( r32( %.15lf ) );\n", box->friction );
        fprintf( file, "\t\tsd.SetRestitution( r32( %.15lf ) );\n", box->restitution );
        fprintf( file, "\t\tsd.SetDensity( r32( %.15lf ) );\n", box->density );
        i32 sensor = (int)box->sensor;
        fprintf( file, "\t\tsd.SetSensor( bool( %d ) );\n", sensor );
        fprintf( file, "\t\tq3Transform boxTx;\n" );
        q3Transform boxTx = box->local;
        q3Vec3 xAxis = boxTx.rotation.ex;
        q3Vec3 yAxis = boxTx.rotation.ey;
        q3Vec3 zAxis = boxTx.rotation.ez;
        fprintf( file, "\t\tq3Vec3 xAxis( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", xAxis.x, xAxis.y, xAxis.z );
        fprintf( file, "\t\tq3Vec3 yAxis( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", yAxis.x, yAxis.y, yAxis.z );
        fprintf( file, "\t\tq3Vec3 zAxis( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", zAxis.x, zAxis.y, zAxis.z );
        fprintf( file, "\t\tboxTx.rotation.SetRows( xAxis, yAxis, zAxis );\n" );
        fprintf( file, "\t\tboxTx.position.Set( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", boxTx.position.x, boxTx.position.y, boxTx.position.z );
        fprintf( file, "\t\tsd.Set( boxTx, q3Vec3( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) ) );\n", box->e.x * 2.0f, box->e.y * 2.0f, box->e.z * 2.0f );
        fprintf( file, "\t\tbodies[ %d ]->AddBox( sd );\n", index );
        fprintf( file, "\t}\n" );
    }

    fprintf( file, "}\n\n" );
}

//--------------------------------------------------------------------------------------------------
void q3BodyRef::CalculateMassData( )
{
    q3Body b = *body();
    q3Mat3 inertia = q3Diagonal( r32( 0.0 ) );
    b.m_invInertiaModel = q3Diagonal( r32( 0.0 ) );
    b.m_invInertiaWorld = q3Diagonal( r32( 0.0 ) );
    b.m_invMass = r32( 0.0 );
    b.m_mass = r32( 0.0 );
    r32 mass = r32( 0.0 );

    if ( b.m_flags & q3Body::eStatic || b.m_flags & q3Body::eKinematic )
    {
        q3Identity( b.m_localCenter );
        b.m_worldCenter = b.m_tx.position;
        return;
    }

    q3Vec3 lc;
    q3Identity( lc );

    for(auto &box : boxes()) {
        if ( box.box()->density == r32( 0.0 ) )
            continue;

        q3MassData md;
        box.ComputeMass( &md );
        mass += md.mass;
        inertia += md.inertia;
        lc += md.center * md.mass;
    }

    if ( mass > r32( 0.0 ) )
    {
        b.m_mass = mass;
        b.m_invMass = r32( 1.0 ) / mass;
        lc *= b.m_invMass;
        q3Mat3 identity;
        q3Identity( identity );
        inertia -= (identity * q3Dot( lc, lc ) - q3OuterProduct( lc, lc )) * mass;
        b.m_invInertiaModel = q3Inverse( inertia );

        if ( b.m_flags & q3Body::eLockAxisX )
            q3Identity( b.m_invInertiaModel.ex );

        if ( b.m_flags & q3Body::eLockAxisY )
            q3Identity( b.m_invInertiaModel.ey );

        if ( b.m_flags & q3Body::eLockAxisZ )
            q3Identity( b.m_invInertiaModel.ez );
    }
    else
    {
        // Force all dynamic bodies to have some mass
        b.m_invMass = r32( 1.0 );
        b.m_invInertiaModel = q3Diagonal( r32( 0.0 ) );
        b.m_invInertiaWorld = q3Diagonal( r32( 0.0 ) );
    }

    b.m_localCenter = lc;
    b.m_worldCenter = q3Mul( b.m_tx, lc );

    *body() = b;
}

//--------------------------------------------------------------------------------------------------
// q3BodyRef
//--------------------------------------------------------------------------------------------------

void q3BodyRef::SynchronizeProxies( )
{
    q3BroadPhase* broadphase = &m_scene->m_contactManager.m_broadphase;

    body()->m_tx.position = body()->m_worldCenter -
        q3Mul( body()->m_tx.rotation, body()->m_localCenter );

    q3AABB aabb;
    q3Transform tx = body()->m_tx;

    for(auto &box : boxes())
    {
        box.ComputeAABB( tx, &aabb );
        broadphase->Update( box.box()->broadPhaseIndex, aabb );
    }
}

std::list<q3BoxRef>& q3BodyRef::boxes() {
    return *m_boxes;
}

const std::list<q3BoxRef>& q3BodyRef::boxes() const {
    return *m_boxes;
}

q3BodyRef::q3BodyRef(q3Scene *scene, q3Container *m_bodyContainer)
    : m_container(m_bodyContainer)
    , m_scene(scene)
{
    m_boxes = new list<q3BoxRef>();
};

q3Body* q3BodyRef::body() const {
   return &m_container->m_bodies[m_bodyIndex];
}

q3BodyRef::~q3BodyRef() {
    delete m_boxes;
}
