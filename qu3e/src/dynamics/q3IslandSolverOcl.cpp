//--------------------------------------------------------------------------------------------------
/**
@file    q3IslandSolverOcl.cpp

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

#include <CL/cl.hpp>
#include <openCLUtilities.hpp>
#include <iostream>
#include <vector>
#include <set>

#include "../scene/q3Scene.h"
#include "q3Body.h"
#include "q3Contact.h"
#include "q3Island.h"
#include "q3IslandSolverOcl.h"


#define assert_size(type, size) assert(sizeof(type) == size)
//#define assert_size(type, size) do { std::cout << "sizeof(" << # type << ") = " << sizeof(type) << std::endl; assert(sizeof(type) == size); } while(0)

// Number of contacts needed for acceleration using OpenCL
#define ACCELERATION_THRESHOLD 64
#define PASSED_ACC_THRESHOLD (/*m_contactCount*/ 0 < ACCELERATION_THRESHOLD)

#define copyBodyInfo(dest, src, sel) do { \
    dest[src->index ## sel].center = src->center ## sel; \
    dest[src->index ## sel].i = src->i ## sel; \
    dest[src->index ## sel].m = src->m ## sel; \
} while(0)

#define CEIL_TO(a, ceil) ((a - 1 + ceil)/ceil) * ceil

cl_int clErr;
#define CHECK_CL_ERROR(info) do { \
        if(clErr) { \
            std::cerr << "OpenCL error: \"" info "\" (" \
                << getCLErrorString(clErr) << ")" << std::endl; \
        } \
    } while(0)

std::string kernelSource =
""
// #include "q3ContactSolverOcl.cl.str"
;

//--------------------------------------------------------------------------------------------------
// q3IslandSolverOcl
//--------------------------------------------------------------------------------------------------

void q3IslandSolverOcl::Solve( q3Scene *scene ) {

    i32 s_bodyCount = scene->m_bodyCount;
    q3Stack *s_stack = &(scene->m_stack);
    q3ContactManager *s_manager = &(scene->m_contactManager);

    m_bodyCapacity = s_bodyCount;

    // q3Body **bodies
    // q3VelocityState *velocities = (q3VelocityState *)s_stack->Allocate( sizeof( q3VelocityState ) * bodyCount );
    // q3ContactConstraint **contacts = (q3ContactConstraint **)s_stack->Allocate( sizeof( q3ContactConstraint* ) * s_manager->m_contactCount );

    m_bodies = (q3Body**) s_stack->Allocate(s_bodyCount);
    m_velocities = (q3VelocityStateOcl *)s_stack->Allocate( sizeof( q3VelocityState ) * s_bodyCount );
    m_contactConstraints = (q3ContactConstraint **)s_stack->Allocate( sizeof( q3ContactConstraint* ) * s_manager->m_contactCount );

//     // Size the stack island, pick worst case size
//     q3Island island;
//     q3Stack *s_stack = &(scene->m_stack);
//     i32 s_bodyCount = scene->m_bodyCount;
//     island.m_bodyCapacity = s_bodyCount;
//     island.m_contactCapacity = scene->m_contactManager.m_contactCount;
//     island.m_bodies = (q3Body**)s_stack->Allocate( sizeof( q3Body* ) * s_bodyCount );
//     island.m_velocities = (q3VelocityState *)s_stack->Allocate( sizeof( q3VelocityState ) * s_bodyCount );
//     island.m_contacts = (q3ContactConstraint **)s_stack->Allocate( sizeof( q3ContactConstraint* ) * island.m_contactCapacity );
//     island.m_contactStates = (q3ContactConstraintState *)s_stack->Allocate( sizeof( q3ContactConstraintState ) * island.m_contactCapacity );
//     island.m_allowSleep = scene->m_allowSleep;
//     island.m_enableFriction = scene->m_enableFriction;
//     island.m_bodyCount = 0;
//     island.m_contactCount = 0;
//     island.m_dt = scene->m_dt;
//     island.m_gravity = scene->m_gravity;
//     island.m_iterations = scene->m_iterations;
//
//
// #ifdef TIMERS_ENABLED
//     struct timeval begin, end, diff;
//     gettimeofday(&begin,NULL);
// #endif // TIMERS_ENABLED
//
//     // Build each active island and then solve each built island
//     i32 stackSize = s_bodyCount;
//     q3Body** stack = (q3Body**)s_stack->Allocate( sizeof( q3Body* ) * stackSize );
//     for ( q3Body* seed = scene->m_bodyList; seed; seed = seed->m_next )
//     {
//         // Seed cannot be apart of an island already
//         if ( seed->m_flags & q3Body::eIsland )
//             continue;
//
//         // Seed must be awake
//         if ( !(seed->m_flags & q3Body::eAwake) )
//             continue;
//
//         // Seed cannot be a static body in order to keep islands
//         // as small as possible
//         if ( seed->m_flags & q3Body::eStatic )
//             continue;
//
//         i32 stackCount = 0;
//         stack[ stackCount++ ] = seed;
//         island.m_bodyCount = 0;
//         island.m_contactCount = 0;
//
//         // Mark seed as apart of island
//         seed->m_flags |= q3Body::eIsland;
//
//         // Perform DFS on constraint graph
//         while( stackCount > 0 )
//         {
//             // Decrement stack to implement iterative backtracking
//             q3Body *body = stack[ --stackCount ];
//             island.Add( body );
//
//             // Awaken all bodies connected to the island
//             body->SetToAwake( );
//
//             // Do not search across static bodies to keep island
//             // formations as small as possible, however the static
//             // body itself should be apart of the island in order
//             // to properly represent a full contact
//             if ( body->m_flags & q3Body::eStatic )
//                 continue;
//
//             // Search all contacts connected to this body
//             q3ContactEdge* contacts = body->m_contactList;
//             for ( q3ContactEdge* edge = contacts; edge; edge = edge->next )
//             {
//                 q3ContactConstraint *contact = edge->constraint;
//
//                 // Skip contacts that have been added to an island already
//                 if ( contact->m_flags & q3ContactConstraint::eIsland )
//                     continue;
//
//                 // Can safely skip this contact if it didn't actually collide with anything
//                 if ( !(contact->m_flags & q3ContactConstraint::eColliding) )
//                     continue;
//
//                 // Skip sensors
//                 if ( contact->A->sensor || contact->B->sensor )
//                     continue;
//
//                 // Mark island flag and add to island
//                 contact->m_flags |= q3ContactConstraint::eIsland;
//                 island.Add( contact );
//
//                 // Attempt to add the other body in the contact to the island
//                 // to simulate contact awakening propogation
//                 q3Body* other = edge->other;
//                 if ( other->m_flags & q3Body::eIsland )
//                     continue;
//
//                 assert( stackCount < stackSize );
//
//                 stack[ stackCount++ ] = other;
//                 other->m_flags |= q3Body::eIsland;
//             }
//         }
//
//         assert( island.m_bodyCount != 0 );
//
//         island.Initialize( );
//
//         assert( island.m_bodyCount != 0 );
//
//         island.Solve( );
//
//         assert( island.m_bodyCount != 0 );
//
//         // Reset all static island flags
//         // This allows static bodies to participate in other island formations
//         for ( i32 i = 0; i < island.m_bodyCount; i++ )
//         {
//             q3Body *body = island.m_bodies[ i ];
//
//             if ( body->m_flags & q3Body::eStatic )
//                 body->m_flags &= ~q3Body::eIsland;
//         }
//     }
//
// #ifdef TIMERS_ENABLED
//     gettimeofday(&end, NULL);
//
//     timersub(&end, &begin, &diff);
//
//     std::cout << "Solve: " << (diff.tv_sec * 1000.0 + diff.tv_usec / 1000.0) << "ms" << std::endl;
// #endif // TIMERS_ENABLED
//
//     s_stack->Free( stack );
//     s_stack->Free( island.m_contactStates );
//     s_stack->Free( island.m_contacts );
//     s_stack->Free( island.m_velocities );
//     s_stack->Free( island.m_bodies );
}

//--------------------------------------------------------------------------------------------------
q3IslandSolverOcl::q3IslandSolverOcl(cl_device_type dev)
{
    assert_size(i32, sizeof(cl_int));
    assert_size(r32, sizeof(cl_float));
    assert_size(q3Vec3, sizeof(cl_float3));
    assert_size(q3Mat3, sizeof(cl_float3) * 3);
    assert_size(q3VelocityState, sizeof(cl_float3) * 2);
    assert_size(q3ContactState, 64);
    // assert_size(q3ContactConstraintState, 720);
    // assert_size(q3ContactPlan, sizeof(cl_int) * 2);


    m_clContext = createCLContext(dev);
    m_clQueue = cl::CommandQueue(m_clContext);

    m_clProgram = buildProgramFromSourceString(m_clContext, kernelSource);

    std::vector<cl::Kernel> kernels;
    m_clProgram.createKernels(&kernels);

    // There is only one kernel
    m_clKernel = kernels.front();
}

//--------------------------------------------------------------------------------------------------
q3IslandSolverOcl::~q3IslandSolverOcl( void )
{
    if(PASSED_ACC_THRESHOLD)
    {
        clErr = m_clQueue.enqueueReadBuffer(*m_clBufferVelocity, true, 0, sizeof(q3VelocityState) * m_bodyCount, m_velocities);
        CHECK_CL_ERROR("Read buffer q3VelocityState");
        clErr = m_clQueue.enqueueReadBuffer(*m_clBufferContactConstraintState, true, 0, sizeof(q3ContactConstraintStateOcl) * m_contactCount, m_contactConstraintStates);
        CHECK_CL_ERROR("Read buffer q3ContactConstraintState");
    }

    // q3ContactSolverCpu::ShutDown();

    m_clGC.deleteAllMemObjects();

    m_clBatches.clear();
    m_clBatchSizes.clear();
}

//--------------------------------------------------------------------------------------------------
void q3IslandSolverOcl::PreSolveContacts( r32 dt )
{
    // q3ContactSolverCpu::PreSolve(dt);

    if(PASSED_ACC_THRESHOLD)
    {
        m_clBufferVelocity = new cl::Buffer(m_clContext, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, sizeof(q3VelocityState) * m_bodyCount, m_velocities, &clErr);
        CHECK_CL_ERROR("Buffer q3VelocityState");
        m_clBufferContactConstraintState = new cl::Buffer(m_clContext, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, sizeof(q3ContactConstraintStateOcl) * m_contactCount, m_contactConstraintStates, &clErr);
        CHECK_CL_ERROR("Buffer q3ContactConstraintStateOcl");

        m_clGC.addMemObject(m_clBufferVelocity);
        m_clGC.addMemObject(m_clBufferContactConstraintState);

        std::vector<unsigned> bodyAllocationTable(m_bodyCount, 0);

        unsigned contactCountTotal = 0;
        std::set<q3ContactPlan> contactsToPlan;
        q3ContactPlan plan;
        for(cl_uint i = 0; i < m_contactCount; i++)
        {
            contactCountTotal += m_contactConstraintStates[i].contactCount;

            plan.contactConstraintStateIndex = i;
            for(cl_uint j = 0; j < m_contactConstraintStates[i].contactCount; j++)
            {
                plan.contactStateIndex = j;
                contactsToPlan.insert(plan);
            }
        }

        m_clBatches.reserve(contactCountTotal);

        cl_uint batchIndex = 1;
        cl_uint batchOffset = 0;
        do
        {
            auto it = contactsToPlan.begin();
            auto end = contactsToPlan.end();
            q3ContactConstraintStateOcl *cc;
            while(it != end) {
                cc = m_contactConstraintStates + it->contactConstraintStateIndex;

                if(bodyAllocationTable[cc->indexA] < batchIndex && bodyAllocationTable[cc->indexB] < batchIndex)
                {
                    bodyAllocationTable[cc->indexA] = batchIndex;
                    bodyAllocationTable[cc->indexB] = batchIndex;

                    m_clBatches.push_back(*it);

                    it = contactsToPlan.erase(it);
                }
                else
                {
                    ++it;
                }
            }

            //  std::cout << "Batch size:" << m_clBatches.size() - batchOffset << std::endl;
            m_clBatchSizes.push_back(m_clBatches.size() - batchOffset);
            batchOffset = m_clBatches.size();

            batchIndex++;

        } while(!contactsToPlan.empty());

        m_clBufferBatches = new cl::Buffer(m_clContext, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(q3ContactPlan) * contactCountTotal, m_clBatches.data(), &clErr);
        CHECK_CL_ERROR("Buffer batches");

        m_clGC.addMemObject(m_clBufferBatches);

        clErr = m_clKernel.setArg(0, *m_clBufferVelocity);
        CHECK_CL_ERROR("Set kernel param 0");
        clErr = m_clKernel.setArg(1, *m_clBufferContactConstraintState);
        CHECK_CL_ERROR("Set kernel param 1");
        clErr = m_clKernel.setArg(2, *m_clBufferBatches);
        CHECK_CL_ERROR("Set kernel param 2");
        clErr = m_clKernel.setArg(5, (cl_int)m_enableFriction);
        CHECK_CL_ERROR("Set kernel param 5");

    }
}

//--------------------------------------------------------------------------------------------------
void q3IslandSolverOcl::SolveContacts( )
{
    cl_uint offset = 0;
    cl::NDRange local(64);
    for(cl_uint batchSize : m_clBatchSizes)
    {
        clErr = m_clKernel.setArg(3, offset);
        CHECK_CL_ERROR("Set kernel param 3");
        clErr = m_clKernel.setArg(4, batchSize);
        CHECK_CL_ERROR("Set kernel param 4");

        offset += batchSize;

        cl::NDRange global(CEIL_TO(batchSize,local[0]));

        clErr = m_clQueue.enqueueNDRangeKernel(m_clKernel, cl::NullRange, global, local);
        CHECK_CL_ERROR("Run kernel");
    }
}

//--------------------------------------------------------------------------------------------------
void q3IslandSolverOcl::Add( q3Body *body )
{
    assert( m_bodyCount < m_bodyCapacity );

    body->m_islandIndex = m_bodyCount;

    m_bodies[ m_bodyCount++ ] = body;
}

//--------------------------------------------------------------------------------------------------
void q3IslandSolverOcl::Add( q3ContactConstraint *contact )
{
    assert( m_contactCount < m_contactCapacity );

    m_contactConstraints[ m_contactCount++ ] = contact;
}
