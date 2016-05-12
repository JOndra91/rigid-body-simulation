//--------------------------------------------------------------------------------------------------
/**
@file    q3IslandSolverCpu.cpp

@author  Ondřej Janošík
@date    13/4/2016

    Copyright (c) 2016 Ondřej Janošík

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
#include <sys/time.h>

#include "q3Body.h"
#include "q3Contact.h"
#include "q3Island.h"
#include "q3IslandSolverCpu.h"
#include "../scene/q3Scene.h"

#include "../debug/q3Timers.h"

//--------------------------------------------------------------------------------------------------
// q3IslandSolverCpu
//--------------------------------------------------------------------------------------------------

void q3IslandSolverCpu::Solve( q3Scene *scene ) {
    // Size the stack island, pick worst case size
    q3Island island;
    q3Stack *s_stack = &(scene->m_stack);
    i32 s_bodyCount = scene->m_bodyCount;
    island.m_bodyCapacity = s_bodyCount;
    island.m_contactCapacity = scene->m_contactManager.m_contactCount;
    island.m_bodies = (q3BodyRef**)s_stack->Allocate( sizeof( q3BodyRef* ) * s_bodyCount );
    island.m_velocities = (q3VelocityState *)s_stack->Allocate( sizeof( q3VelocityState ) * s_bodyCount );
    island.m_contacts = (q3ContactConstraint **)s_stack->Allocate( sizeof( q3ContactConstraint* ) * island.m_contactCapacity );
    island.m_contactStates = (q3ContactConstraintState *)s_stack->Allocate( sizeof( q3ContactConstraintState ) * island.m_contactCapacity );
    island.m_allowSleep = scene->m_allowSleep;
    island.m_enableFriction = scene->m_enableFriction;
    island.m_bodyCount = 0;
    island.m_contactCount = 0;
    island.m_dt = scene->m_dt;
    island.m_gravity = scene->m_gravity;
    island.m_iterations = scene->m_iterations;

    // Build each active island and then solve each built island
    i32 stackSize = s_bodyCount;
    q3BodyRef** stack = (q3BodyRef**)s_stack->Allocate( sizeof( q3Body* ) * stackSize );
    for ( auto seedRef : scene->m_container.bodies() )
    {
        q3Body *seed = seedRef->body();
        // Seed cannot be apart of an island already
        if ( seed->m_flags & q3Body::eIsland )
            continue;

        // Seed must be awake
        if ( !(seed->m_flags & q3Body::eAwake) )
            continue;

        // Seed cannot be a static body in order to keep islands
        // as small as possible
        if ( seed->m_flags & q3Body::eStatic )
            continue;

        i32 stackCount = 0;
        stack[ stackCount++ ] = seedRef;
        island.m_bodyCount = 0;
        island.m_contactCount = 0;

        // Mark seed as apart of island
        seed->m_flags |= q3Body::eIsland;

        // Perform DFS on constraint graph
        while( stackCount > 0 )
        {
            // Decrement stack to implement iterative backtracking
            q3BodyRef *body = stack[ --stackCount ];
            island.Add( body );

            // Awaken all bodies connected to the island
            body->SetToAwake( );

            // Do not search across static bodies to keep island
            // formations as small as possible, however the static
            // body itself should be apart of the island in order
            // to properly represent a full contact
            if ( body->body()->m_flags & q3Body::eStatic )
                continue;

            // Search all contacts connected to this body
            q3ContactEdge* contacts = body->m_contactList;
            for ( q3ContactEdge* edge = contacts; edge; edge = edge->next )
            {
                q3ContactConstraint *contact = edge->constraint;

                // Skip contacts that have been added to an island already
                if ( contact->m_flags & q3ContactConstraint::eIsland )
                    continue;

                // Can safely skip this contact if it didn't actually collide with anything
                if ( !(contact->m_flags & q3ContactConstraint::eColliding) )
                    continue;

                // Skip sensors
                if ( contact->A->box()->sensor || contact->B->box()->sensor )
                    continue;

                // Mark island flag and add to island
                contact->m_flags |= q3ContactConstraint::eIsland;
                island.Add( contact );

                // Attempt to add the other body in the contact to the island
                // to simulate contact awakening propogation
                q3BodyRef* other = edge->other;
                if ( other->body()->m_flags & q3Body::eIsland )
                    continue;

                assert( stackCount < stackSize );

                stack[ stackCount++ ] = other;
                other->body()->m_flags |= q3Body::eIsland;
            }
        }

        assert( island.m_bodyCount != 0 );

        island.Initialize( );

        assert( island.m_bodyCount != 0 );

        island.Solve( );

        assert( island.m_bodyCount != 0 );

        // Reset all static island flags
        // This allows static bodies to participate in other island formations
        for ( i32 i = 0; i < island.m_bodyCount; i++ )
        {
            q3Body *body = island.m_bodies[ i ]->body();

            if ( body->m_flags & q3Body::eStatic )
                body->m_flags &= ~q3Body::eIsland;
        }
    }

    q3TimerPrint("solve", "  Solve");
    q3TimerClear("solve");

    s_stack->Free( stack );
    s_stack->Free( island.m_contactStates );
    s_stack->Free( island.m_contacts );
    s_stack->Free( island.m_velocities );
    s_stack->Free( island.m_bodies );

}
