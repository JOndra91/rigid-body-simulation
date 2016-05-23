//--------------------------------------------------------------------------------------------------
/**
@file    q3Scene.h

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

#include <stdlib.h>
#include <iostream>

#include "q3Scene.h"
#include "../common/q3OpenCL.h"
#include "../debug/q3Timers.h"
#include "../dynamics/q3Body.h"
#include "../dynamics/q3Contact.h"
#include "../dynamics/q3IslandSolverCpu.h"
#include "../dynamics/q3IslandSolverOcl.h"
#include "../dynamics/q3ContactSolverCpu.h"
#include "../dynamics/q3ContactManagerOcl.h"
#include "../collision/q3Box.h"

//--------------------------------------------------------------------------------------------------
// q3Scene
//--------------------------------------------------------------------------------------------------
q3Scene::q3Scene( r32 dt, q3OpenCLDevice device, const q3Vec3& gravity, i32 iterations )
    : m_boxAllocator( sizeof( q3Box ), 256 )
    , m_bodyCount( 0 )
    , m_container(this)
    , m_gravity( gravity )
    , m_dt( dt )
    , m_iterations( iterations )
    , m_newBox( false )
    , m_allowSleep( true )
    , m_enableFriction( true )
{

    if(q3IsOpenCLAccelerated(device)) {
        cl_device_type clDevType;
        switch(device) {
            case q3OpenCLDevice::CPU:
                clDevType = CL_DEVICE_TYPE_CPU;
                break;
            case q3OpenCLDevice::GPU:
                clDevType = CL_DEVICE_TYPE_GPU;
                break;
            default:
                clDevType = CL_DEVICE_TYPE_DEFAULT;
        }
        m_clContext = createCLContext(clDevType);

        m_contactManager = new q3ContactManager(&m_stack);
        // v This doesn't work :(
        // m_contactManager = new q3ContactManagerOcl(&m_stack, &m_container, &m_clContext);
        m_islandSolver = new q3IslandSolverOcl(&m_container, &m_clContext);
        // v Use for consistent testing with OpenCL contact manager
        // m_islandSolver = new q3IslandSolverCpu();
    }
    else {
        m_contactManager = new q3ContactManager(&m_stack);
        m_islandSolver = new q3IslandSolverCpu();
    }
}

//--------------------------------------------------------------------------------------------------
q3Scene::~q3Scene( )
{
    Shutdown( );

    delete m_islandSolver;
}

//--------------------------------------------------------------------------------------------------
void q3Scene::Step( )
{
    q3TimerStart("step");
    // q3TimerStart("subStep");

    if ( m_newBox )
    {
        m_contactManager->m_broadphase.UpdatePairs( );
        m_newBox = false;
    }

    // q3TimerStop("subStep");
    // q3TimerPrint("subStep", " Update pairs");
    // q3TimerStart("subStep");

    m_contactManager->TestCollisions( );

    // q3TimerStop("subStep");
    // q3TimerPrint("subStep", " Test collisions");
    // q3TimerStart("subStep");

    for(auto &body : m_container.m_bodies) {
        body.m_flags = body.m_flags & ~q3Body::eIsland;;
    }

    for ( q3ContactConstraint* c = m_contactManager->m_contactList; c; c = c->next )
        c->m_flags &= ~q3ContactConstraint::eIsland;

    // q3TimerStop("subStep");
    // q3TimerPrint("subStep", " Reset flags");
    q3TimerStart("subStep");

    m_islandSolver->Solve(this);

    q3TimerStop("subStep");
    q3TimerPrint("subStep", " Island solver");
    // q3TimerStart("subStep");

    // Update the broadphase AABBs
    for(auto body : m_container.m_bodyPtrs) {
        if ( body->body()->m_flags & q3Body::eStatic )
            continue;

        body->SynchronizeProxies( );
    }

    // q3TimerStop("subStep");
    // q3TimerPrint("subStep", " Synchronize proxies");
    // q3TimerStart("subStep");

    // Look for new contacts
    m_contactManager->FindNewContacts( );

    // q3TimerStop("subStep");
    // q3TimerPrint("subStep", " Find new contacts");
    // q3TimerStart("subStep");

    // Clear all forces
    for(auto &body : m_container.m_bodies) {
        q3Identity( body.m_force );
        q3Identity( body.m_torque );
    }

    // q3TimerStop("subStep");
    q3TimerStop("step");
    // q3TimerPrint("subStep", " Clear forces");
    q3TimerPrint("step", "Step");
    #ifdef TIMERS_ENABLED
        std::cout << "====================================" << std::endl;
    #endif // TIMERS_ENABLED
}

//--------------------------------------------------------------------------------------------------
q3BodyRef* q3Scene::CreateBody( const q3BodyDef& def )
{
    ++m_bodyCount;
    return m_container.create(def);
}

//--------------------------------------------------------------------------------------------------
void q3Scene::RemoveBody( q3BodyRef *body )
{
    --m_bodyCount;
    m_container.remove(body);
}

//--------------------------------------------------------------------------------------------------
void q3Scene::RemoveAllBodies( )
{
    m_bodyCount = 0;
    m_container.clear();
}

//--------------------------------------------------------------------------------------------------
void q3Scene::SetAllowSleep( bool allowSleep )
{
    m_allowSleep = allowSleep;

    if ( !allowSleep )
    {
        for(auto body : m_container.bodies()) {
            body->SetToAwake( );
        }
    }
}

//--------------------------------------------------------------------------------------------------
void q3Scene::SetIterations( i32 iterations )
{
    m_iterations = q3Max( 1, iterations );
}

//--------------------------------------------------------------------------------------------------
void q3Scene::SetEnableFriction( bool enabled )
{
    m_enableFriction = enabled;
}

//--------------------------------------------------------------------------------------------------
void q3Scene::Render( q3Render* render ) const
{
    for (const auto body : m_container.bodies())
    {
        body->Render( render );
    }

    m_contactManager->RenderContacts( render );
}
//--------------------------------------------------------------------------------------------------
void q3Scene::RenderBroadphase( q3Render* render ) const
{
    m_contactManager->m_broadphase.m_tree.Render( render );
}

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Scene::GetGravity( ) const
{
    return m_gravity;
}

//--------------------------------------------------------------------------------------------------
void q3Scene::SetGravity( const q3Vec3& gravity )
{
    m_gravity = gravity;
}

//--------------------------------------------------------------------------------------------------
void q3Scene::Shutdown( )
{
    RemoveAllBodies( );

    m_boxAllocator.Clear( );
}

//--------------------------------------------------------------------------------------------------
void q3Scene::SetContactListener( q3ContactListener* listener )
{
    m_contactManager->m_contactListener = listener;
}

//--------------------------------------------------------------------------------------------------
void q3Scene::QueryAABB( q3QueryCallback *cb, const q3AABB& aabb ) const
{
    struct SceneQueryWrapper
    {
        bool TreeCallBack( i32 id )
        {
            q3AABB aabb;
            q3BoxRef *box = (q3BoxRef *)broadPhase->m_tree.GetUserData( id );

            box->ComputeAABB( box->body()->GetTransform( ), &aabb );

            if ( q3AABBtoAABB( m_aabb, aabb ) )
            {
                return cb->ReportShape( box->box() );
            }

            return true;
        }

        q3QueryCallback *cb;
        const q3BroadPhase *broadPhase;
        q3AABB m_aabb;
    };

    SceneQueryWrapper wrapper;
    wrapper.m_aabb = aabb;
    wrapper.broadPhase = &m_contactManager->m_broadphase;
    wrapper.cb = cb;
    m_contactManager->m_broadphase.m_tree.Query( &wrapper, aabb );
}

//--------------------------------------------------------------------------------------------------
void q3Scene::QueryPoint( q3QueryCallback *cb, const q3Vec3& point ) const
{
    struct SceneQueryWrapper
    {
        bool TreeCallBack( i32 id )
        {
            q3BoxRef *box = (q3BoxRef *)broadPhase->m_tree.GetUserData( id );

            if ( box->TestPoint( box->body()->GetTransform( ), m_point ) )
            {
                cb->ReportShape( box->box() );
            }

            return true;
        }

        q3QueryCallback *cb;
        const q3BroadPhase *broadPhase;
        q3Vec3 m_point;
    };

    SceneQueryWrapper wrapper;
    wrapper.m_point = point;
    wrapper.broadPhase = &m_contactManager->m_broadphase;
    wrapper.cb = cb;
    const r32 k_fattener = r32( 0.5 );
    q3Vec3 v( k_fattener, k_fattener, k_fattener );
    q3AABB aabb;
    aabb.min = point - v;
    aabb.max = point + v;
    m_contactManager->m_broadphase.m_tree.Query( &wrapper, aabb );
}

//--------------------------------------------------------------------------------------------------
void q3Scene::RayCast( q3QueryCallback *cb, q3RaycastData& rayCast ) const
{
    struct SceneQueryWrapper
    {
        bool TreeCallBack( i32 id )
        {
            q3BoxRef *box = (q3BoxRef *)broadPhase->m_tree.GetUserData( id );

            if ( box->Raycast( box->body()->GetTransform( ), m_rayCast ) )
            {
                return cb->ReportShape( box->box() );
            }

            return true;
        }

        q3QueryCallback *cb;
        const q3BroadPhase *broadPhase;
        q3RaycastData *m_rayCast;
    };

    SceneQueryWrapper wrapper;
    wrapper.m_rayCast = &rayCast;
    wrapper.broadPhase = &m_contactManager->m_broadphase;
    wrapper.cb = cb;
    m_contactManager->m_broadphase.m_tree.Query( &wrapper, rayCast );
}

//--------------------------------------------------------------------------------------------------
void q3Scene::Dump( FILE* file ) const
{
    fprintf( file, "// Ensure 64/32-bit memory compatability with the dump contents\n" );
    fprintf( file, "assert( sizeof( int* ) == %lu );\n", sizeof( int* ) );
    fprintf( file, "scene.SetGravity( q3Vec3( %.15lf, %.15lf, %.15lf ) );\n", m_gravity.x, m_gravity.y, m_gravity.z );
    fprintf( file, "scene.SetAllowSleep( %s );\n", m_allowSleep ? "true" : "false" );
    fprintf( file, "scene.SetEnableFriction( %s );\n", m_enableFriction ? "true" : "false" );

    fprintf( file, "q3Body** bodies = (q3Body**)q3Alloc( sizeof( q3Body* ) * %d );\n", m_bodyCount );

    i32 i = 0;
    for(auto body : m_container.bodies()) {
        body->Dump(file, i++);
    }

    fprintf( file, "q3Free( bodies );\n" );
}
