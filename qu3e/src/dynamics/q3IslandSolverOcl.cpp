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

#include "../scene/q3Scene.h"
#include "q3Body.h"
#include "q3Contact.h"
#include "q3Island.h"
#include "q3IslandSolverOcl.h"

#include "../debug/q3Timers.h"

#ifdef NO_KERNEL_SOURCE
std::string solverKernelSource = "";
#else
std::string solverKernelSource =
#include "q3ContactSolverOcl.cl.str"
;
#endif // NO_KERNEL_SOURCE

//--------------------------------------------------------------------------------------------------
// q3IslandSolverOcl
//--------------------------------------------------------------------------------------------------
q3IslandSolverOcl::q3IslandSolverOcl(q3Container *container, cl::Context *ctx)
{
    // printf("sizeof(i32) = %lu\n", sizeof(i32));
    // printf("sizeof(r32) = %lu\n", sizeof(r32));
    // printf("sizeof(u32) = %lu\n", sizeof(u32));
    // printf("sizeof(q3Vec3) = %lu\n", sizeof(q3Vec3));
    // printf("sizeof(q3Mat3) = %lu\n", sizeof(q3Mat3));
    // printf("sizeof(q3VelocityStateOcl) = %lu\n", sizeof(q3VelocityStateOcl));
    // printf("sizeof(q3ContactStateOcl) = %lu\n", sizeof(q3ContactStateOcl));
    // printf("sizeof(q3ContactConstraintStateOcl) = %lu\n", sizeof(q3ContactConstraintStateOcl));

    assert_size(i32, sizeof(cl_int));
    assert_size(r32, sizeof(cl_float));
    assert_size(q3Vec3, 16);
    assert_size(q3Vec3, sizeof(cl_float3));
    assert_size(q3Quaternion, 16);
    assert_size(q3Quaternion, sizeof(cl_float4));
    assert_size(q3Mat3, 48);
    assert_size(q3Mat3, sizeof(cl_float3) * 3);
    assert_size(q3VelocityStateOcl, 32);
    assert_size(q3VelocityStateOcl, sizeof(cl_float3) * 2);
    assert_size(q3ContactStateOcl, 80);
    assert_size(q3ContactConstraintStateOcl, 208);
    assert_size(q3Body, 320);

    m_container = container;
    m_clContext = ctx;
    m_clQueue = cl::CommandQueue(*m_clContext);

    // m_clProgram = buildProgramFromSourceString(*m_clContext, solverKernelSource);
    m_clProgram = buildProgramFromSource(*m_clContext, "./qu3e/kernels/q3ContactSolverOcl.cl");

    std::vector<cl::Kernel> kernels;
    m_clProgram.createKernels(&kernels);

    assert(kernels.size() == 4);

    for(auto &kernel : kernels) {
        std::string name = kernel.getInfo<CL_KERNEL_FUNCTION_NAME>();
        if(name == "preSolve") {
            m_clKernelPreSolve = kernel;
        }
        else if(name == "solve") {
            m_clKernelSolve = kernel;
        }
        else if(name == "prepare") {
            m_clKernelPrepare = kernel;
        }
        else if(name == "integrate") {
            m_clKernelIntegrate = kernel;
        }
    }

    std::vector<cl::Device> devices;

    m_clContext->getInfo(CL_CONTEXT_DEVICES, &devices);

    m_clLocalSize = m_clKernelSolve.getWorkGroupInfo
        <CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE>(devices.front());
}

//--------------------------------------------------------------------------------------------------
q3IslandSolverOcl::~q3IslandSolverOcl( void ) {
    m_clGC.deleteAllMemObjects();
}

//--------------------------------------------------------------------------------------------------
void q3IslandSolverOcl::Solve( q3Scene *scene ) {

    cl_int clErr;
    i32 s_bodyCount = scene->m_bodyCount;
    q3Stack *s_stack = &(scene->m_stack);
    q3ContactManager *s_manager = scene->m_contactManager;

    m_scene = scene;

    m_bodyCapacity = s_bodyCount + 1;
    m_bodyCount = 0;
    m_contactCount = 0;
    m_contactStateCount = 0;
    m_contactCapacity = s_manager->m_contactCount;

    m_bodies = (q3BodyRef**) s_stack->Allocate( sizeof(q3BodyRef*) * m_bodyCapacity);
    m_contactConstraints = (q3ContactConstraint **)s_stack->Allocate( sizeof( q3ContactConstraint* ) * m_contactCapacity );

    // Build each active island and then solve each built island
    i32 stackSize = s_bodyCount;
    q3BodyRef** stack = (q3BodyRef**)s_stack->Allocate( sizeof( q3Body* ) * stackSize );
    m_islandId = 0;
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

        // Mark seed as apart of island
        seed->m_flags |= q3Body::eIsland;

        // Perform DFS on constraint graph
        while( stackCount > 0 )
        {
            // Decrement stack to implement iterative backtracking
            q3BodyRef *body = stack[ --stackCount ];
            Add( body );

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
                Add( contact );

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
        m_islandId++;
    }

    if(m_bodyCount == 0) {
        return;
    }

    // If this happens, add 1 to prevent empty cl::Buffer initialization.
    assert(m_islandId != 0);

    // Apply gravity
    // Integrate velocities and create state buffers, calculate world inertia
    r32 dt = m_scene->m_dt;

#if 0
#define DUMP_FILE "cpu.dump"

    q3Vec3 gravity = m_scene->m_gravity;
    for ( i32 i = 0 ; i < m_bodyCount; ++i )
    {
        q3BodyRef* bodyRef = m_bodies[ i ];
        q3Body *body = bodyRef->body();
        q3VelocityStateOcl *v = m_velocities + i;

        assert(i == body->m_islandIndex);

        if ( body->m_flags & q3Body::eDynamic )
        {
            body->ApplyLinearForce( gravity * body->m_gravityScale );

            // Calculate world space intertia tensor
            q3Mat3 r = body->m_tx.rotation;
            body->m_invInertiaWorld = r * body->m_invInertiaModel * q3Transpose( r );

            // Integrate velocity
            body->m_linearVelocity += (body->m_force * body->m_invMass) * dt;
            body->m_angularVelocity += (body->m_invInertiaWorld * body->m_torque) * dt;

            // From Box2D!
            // Apply damping.
            // ODE: dv/dt + c * v = 0
            // Solution: v(t) = v0 * exp(-c * t)
            // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
            // v2 = exp(-c * dt) * v1
            // Pade approximation:
            // v2 = v1 * 1 / (1 + c * dt)
            body->m_linearVelocity *= r32( 1.0 ) / (r32( 1.0 ) + dt * r32( 0.0 ));
            body->m_angularVelocity *= r32( 1.0 ) / (r32( 1.0 ) + dt * r32( 0.1 ));
        }

        v->v = body->m_linearVelocity;
        v->w = body->m_angularVelocity;
    }


#else
#define DUMP_FILE "ocl.dump"

    cl::Buffer clBufferIndicies = cl::Buffer(*m_clContext, CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR, sizeof(cl_uint) * m_bodyCount, NULL, &clErr);
    CHECK_CL_ERROR(clErr, "Buffer indicies");

    cl_uint *indicies = (cl_uint*)m_clQueue.enqueueMapBuffer(clBufferIndicies, CL_TRUE, CL_MAP_WRITE, 0, sizeof(cl_uint) * m_bodyCount, NULL, NULL, &clErr);
    CHECK_CL_ERROR(clErr, "Map indicies");

    for ( i32 i = 0 ; i < m_bodyCount; ++i ) {
        indicies[i] = m_bodies[i]->getBodyIndex();
    }

    clErr = m_clQueue.enqueueUnmapMemObject(clBufferIndicies, indicies);
    CHECK_CL_ERROR(clErr, "Unmap indicies");

    cl::Buffer clBufferBody = cl::Buffer(*m_clContext, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, sizeof(q3Body) * m_container->m_bodies.size(), m_container->m_bodies.data(), &clErr);
    CHECK_CL_ERROR(clErr, "Buffer q3Body");

    m_clBufferVelocity = new cl::Buffer(*m_clContext, CL_MEM_READ_WRITE, sizeof(q3VelocityStateOcl) * m_bodyCount, NULL, &clErr);
    CHECK_CL_ERROR(clErr, "Buffer q3VelocityStateOcl");
    m_clGC.addMemObject(m_clBufferVelocity);


    clErr = m_clKernelPrepare.setArg(0, clBufferBody);
    CHECK_CL_ERROR(clErr, "Set prepare kernel param 0 (body)");
    clErr = m_clKernelPrepare.setArg(1, *m_clBufferVelocity);
    CHECK_CL_ERROR(clErr, "Set prepare kernel param 1 (velocity)");
    clErr = m_clKernelPrepare.setArg(2, (cl_float)dt);
    CHECK_CL_ERROR(clErr, "Set prepare kernel param 2 (dt)");
    clErr = m_clKernelPrepare.setArg(3, m_scene->m_gravity);
    CHECK_CL_ERROR(clErr, "Set prepare kernel param 3 (gravity)");
    clErr = m_clKernelPrepare.setArg(4, clBufferIndicies );
    CHECK_CL_ERROR(clErr, "Set prepare kernel param 4 (indicies)");
    clErr = m_clKernelPrepare.setArg(5, m_bodyCount);
    CHECK_CL_ERROR(clErr, "Set prepare kernel param 5 (body count)");

    cl::NDRange local(m_clLocalSize);
    cl::NDRange global(CEIL_TO(m_bodyCount,local[0]));
    clErr = m_clQueue.enqueueNDRangeKernel(m_clKernelPrepare, cl::NullRange, global, local);
    CHECK_CL_ERROR(clErr, "Run prepare kernel");

#endif

#if 0
    FILE *f = fopen(DUMP_FILE, "a");

    for ( i32 i = 0 ; i < m_bodyCount; ++i )
    {
        q3BodyRef* bodyRef = m_bodies[ i ];
        q3Body *body = bodyRef->body();
        q3VelocityStateOcl *v = m_velocities + i;

        fprintf(f, "Body[%d]:\n", i);
        fprintf(f, "  linear velocity: (%.9f, %.9f, %.9f)\n", v->v.x, v->v.y, v->v.z);
        fprintf(f, "  angular velocity: (%.9f, %.9f, %.9f)\n", v->w.x, v->w.y, v->w.z);
        fprintf(f, "  inertia: [[%.9f, %.9f, %.9f,], [%.9f, %.9f, %.9f,], [%.9f, %.9f, %.9f,]]\n",
            body->m_invInertiaWorld.ex.x, body->m_invInertiaWorld.ex.y, body->m_invInertiaWorld.ex.z,
            body->m_invInertiaWorld.ey.x, body->m_invInertiaWorld.ey.y, body->m_invInertiaWorld.ey.z,
            body->m_invInertiaWorld.ez.x, body->m_invInertiaWorld.ez.y, body->m_invInertiaWorld.ez.z
        );
    }

    fprintf(f, "------------------------------------------------\n\n");

    fclose(f);
#endif

    m_contactStates = NULL;
    m_contactConstraintStates = NULL;

    q3TimerStart("solve");

    if(m_contactCount > 0) {

        m_clBufferContactState = new cl::Buffer(*m_clContext, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR, sizeof(q3ContactStateOcl) * m_contactStateCount, NULL, &clErr);
        CHECK_CL_ERROR(clErr, "Buffer q3ContactStateOcl");

        m_clBufferContactConstraintState = new cl::Buffer(*m_clContext, CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR, sizeof(q3ContactConstraintStateOcl) * m_contactCount, NULL, &clErr);
        CHECK_CL_ERROR(clErr, "Buffer q3ContactConstraintStateOcl");

        m_contactStates = (q3ContactStateOcl*)m_clQueue.enqueueMapBuffer(*m_clBufferContactState, CL_TRUE, CL_MAP_WRITE, 0, sizeof(q3ContactStateOcl) * m_contactStateCount, NULL, NULL, &clErr);
        CHECK_CL_ERROR(clErr, "Map q3ContactStateOcl");

        m_contactConstraintStates = (q3ContactConstraintStateOcl*)m_clQueue.enqueueMapBuffer(*m_clBufferContactConstraintState, CL_TRUE, CL_MAP_WRITE, 0, sizeof(q3ContactConstraintStateOcl) * m_contactCount, NULL, NULL, &clErr);
        CHECK_CL_ERROR(clErr, "Map q3ContactConstraintStateOcl");

        m_constraintIndicies = (u32*) s_stack->Allocate( sizeof(u32) * m_contactStateCount);
        m_constraintPairs = (constraintPair*) s_stack->Allocate( sizeof(constraintPair) * m_contactCount);

        InitializeContacts();

        clErr = m_clQueue.enqueueUnmapMemObject(*m_clBufferContactState, m_contactStates);
        m_contactStates = NULL;
        CHECK_CL_ERROR(clErr, "Unmap q3ContactStateOcl");

        clErr = m_clQueue.enqueueUnmapMemObject(*m_clBufferContactConstraintState, m_contactConstraintStates);
        m_contactConstraintStates = NULL;
        CHECK_CL_ERROR(clErr, "Unmap q3ContactStateOcl");


        m_clGC.addMemObject(m_clBufferContactState);
        m_clGC.addMemObject(m_clBufferContactConstraintState);

        std::list<cl_uint> contactsToPlan;
        std::vector<cl_uint> bodyAllocationTable(m_bodyCount, 0);

        for(cl_uint i = 0; i < m_contactStateCount; ++i) {
            contactsToPlan.push_back(i);
        }

        m_clBatches.reserve(m_contactStateCount);

        cl_uint batchIndex = 1;
        cl_uint batchOffset = 0;
        cl_uint lastConstraintIndex = UINT32_MAX;
        do
        {
            for(auto it = contactsToPlan.begin(); it != contactsToPlan.end();) {
                cl_uint constraintIndex = m_constraintIndicies[*it];
                constraintPair *cc = m_constraintPairs + constraintIndex;

                if(lastConstraintIndex == constraintIndex) {
                    ++m_clBatches.back().s[1];
                    it = contactsToPlan.erase(it);
                }
                else if(
                     (bodyAllocationTable[cc->indexA] < batchIndex || (m_bodies[cc->indexA]->body()->m_flags & q3Body::eStatic))
                  && (bodyAllocationTable[cc->indexB] < batchIndex || (m_bodies[cc->indexB]->body()->m_flags & q3Body::eStatic)))
                {

                    bodyAllocationTable[cc->indexA] = batchIndex;
                    bodyAllocationTable[cc->indexB] = batchIndex;

                    cl_uint2 v = {.s = {*it, 1}};
                    m_clBatches.push_back(v);

                    it = contactsToPlan.erase(it);
                    lastConstraintIndex = constraintIndex;
                }
                else {
                    ++it;
                }
            }

            //  std::cout << "Batch size:" << m_clBatches.size() - batchOffset << std::endl;
            m_clBatchSizes.push_back(m_clBatches.size() - batchOffset);
            batchOffset = m_clBatches.size();

            batchIndex++;

        } while(!contactsToPlan.empty());

        s_stack->Free(m_constraintPairs);
        s_stack->Free(m_constraintIndicies);

        m_clBufferBatches = new cl::Buffer(*m_clContext, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(cl_uint2) * m_clBatches.size(), m_clBatches.data(), &clErr);
        CHECK_CL_ERROR(clErr, "Buffer batches");

        m_clGC.addMemObject(m_clBufferBatches);

        PreSolveContacts();
        SolveContacts();
    }

    q3TimerStop("solve");
    q3TimerPrint("solve", "  Solve");

    cl::Buffer clBufferIslandMin = cl::Buffer(*m_clContext, CL_MEM_READ_WRITE, sizeof(cl_float) * m_islandId, NULL, &clErr);
    CHECK_CL_ERROR(clErr, "Read buffer island min");

    // Integrate positions
    clErr = m_clKernelIntegrate.setArg(0, clBufferBody);
    CHECK_CL_ERROR(clErr, "Set integrate kernel param 0 (body)");
    clErr = m_clKernelIntegrate.setArg(1, *m_clBufferVelocity);
    CHECK_CL_ERROR(clErr, "Set integrate kernel param 1 (velocity)");
    clErr = m_clKernelIntegrate.setArg(2, (cl_float)dt);
    CHECK_CL_ERROR(clErr, "Set integrate kernel param 2 (dt)");
    clErr = m_clKernelIntegrate.setArg(3, clBufferIndicies);
    CHECK_CL_ERROR(clErr, "Set integrate kernel param 3 (indicies)");
    clErr = m_clKernelIntegrate.setArg(4, m_bodyCount);
    CHECK_CL_ERROR(clErr, "Set integrate kernel param 4 (body count)");
    clErr = m_clKernelIntegrate.setArg(5, (cl_uint)m_scene->m_allowSleep);
    CHECK_CL_ERROR(clErr, "Set integrate kernel param 5 (allow sleep)");
    clErr = m_clKernelIntegrate.setArg(6, (cl_float)Q3_SLEEP_TIME);
    CHECK_CL_ERROR(clErr, "Set integrate kernel param 6 (sleep time)");
    clErr = m_clKernelIntegrate.setArg(7, clBufferIslandMin);
    CHECK_CL_ERROR(clErr, "Set integrate kernel param 7 (clBufferIslandMin)");

    global = cl::NDRange(CEIL_TO(m_bodyCount,local[0]));
    clErr = m_clQueue.enqueueNDRangeKernel(m_clKernelIntegrate, cl::NullRange, global, local);
    CHECK_CL_ERROR(clErr, "Run integrate kernel");

    if(m_contactCount > 0) {

        m_contactStates = (q3ContactStateOcl*)m_clQueue.enqueueMapBuffer(*m_clBufferContactState, CL_TRUE, CL_MAP_READ, 0, sizeof(q3ContactStateOcl) * m_contactStateCount, NULL, NULL, &clErr);
        CHECK_CL_ERROR(clErr, "Map q3ContactStateOcl");

        q3ContactStateOcl *cs = m_contactStates;
        for ( i32 i = 0; i < m_contactCount; ++i )
        {
            q3ContactConstraint *cc = m_contactConstraints[ i ];

            for ( i32 j = 0; j < cc->manifold.contactCount; ++j )
            {
                q3Contact *oc = cc->manifold.contacts + j;
                oc->normalImpulse = cs->normalImpulse;
                oc->tangentImpulse[ 0 ] = cs->tangentImpulse[ 0 ];
                oc->tangentImpulse[ 1 ] = cs->tangentImpulse[ 1 ];

                cs++;
            }
        }

        clErr = m_clQueue.enqueueUnmapMemObject(*m_clBufferContactState, m_contactStates);
        m_contactStates = NULL;
        CHECK_CL_ERROR(clErr, "Unmap q3ContactStateOcl");
    }

#if 0
    for ( i32 i = 0 ; i < m_bodyCount; ++i )
    {
        q3BodyRef* bodyRef = m_bodies[ i ];
        q3Body *body = bodyRef->body();
        q3VelocityStateOcl *v = m_velocities + i;

        if ( body->m_flags & q3Body::eStatic )
            continue;

        body->m_linearVelocity = v->v;
        body->m_angularVelocity = v->w;

        // Integrate position
        body->m_worldCenter += body->m_linearVelocity * dt;
        body->m_q.Integrate( body->m_angularVelocity, dt );
        body->m_q = q3Normalize( body->m_q );
        body->m_tx.rotation = body->m_q.ToMat3( );
    }
#endif
#if 0
    if ( m_scene->m_allowSleep )
    {
        // Find minimum sleep time of the entire island
        f32 minSleepTime = Q3_R32_MAX;
        for ( i32 i = 0; i < m_bodyCount; ++i )
        {
            q3BodyRef* bodyRef = m_bodies[ i ];
            q3Body *body = bodyRef->body();

            if ( body->m_flags & q3Body::eStatic )
                continue;

            const r32 sqrLinVel = q3Dot( body->m_linearVelocity, body->m_linearVelocity );
            const r32 cbAngVel = q3Dot( body->m_angularVelocity, body->m_angularVelocity );
            const r32 linTol = Q3_SLEEP_LINEAR;
            const r32 angTol = Q3_SLEEP_ANGULAR;

            if ( sqrLinVel > linTol || cbAngVel > angTol )
            {
                minSleepTime = r32( 0.0 );
                body->m_sleepTime = r32( 0.0 );
            }

            else
            {
                body->m_sleepTime += dt;
                minSleepTime = q3Min( minSleepTime, body->m_sleepTime );
            }
        }

        // Put entire island to sleep so long as the minimum found sleep time
        // is below the threshold. If the minimum sleep time reaches below the
        // sleeping threshold, the entire island will be reformed next step
        // and sleep test will be tried again.
        if ( minSleepTime > Q3_SLEEP_TIME )
        {
            for ( i32 i = 0; i < m_bodyCount; ++i )
                m_bodies[ i ]->SetToSleep( );
        }
    }
#endif

    clErr = m_clQueue.enqueueReadBuffer(clBufferBody, CL_TRUE, 0, sizeof(q3Body) * m_container->m_bodies.size(), m_container->m_bodies.data());
    CHECK_CL_ERROR(clErr, "Read body buffer");

    m_clQueue.finish();

    s_stack->Free(stack);
    s_stack->Free(m_contactConstraints);
    s_stack->Free(m_bodies);

    m_clGC.deleteAllMemObjects();

    m_clBatches.clear();
    m_clBatchSizes.clear();
}

//--------------------------------------------------------------------------------------------------
void q3IslandSolverOcl::PreSolveContacts()
{
    cl_int clErr;
    clErr = m_clKernelPreSolve.setArg(0, *m_clBufferVelocity);
    CHECK_CL_ERROR(clErr, "Set pre-solve kernel param 0 (velocity)");
    clErr = m_clKernelPreSolve.setArg(1, *m_clBufferContactConstraintState);
    CHECK_CL_ERROR(clErr, "Set pre-solve kernel param 1 (contact constraint state)");
    clErr = m_clKernelPreSolve.setArg(2, *m_clBufferContactState);
    CHECK_CL_ERROR(clErr, "Set pre-solve kernel param 2 (contact state)");
    clErr = m_clKernelPreSolve.setArg(3, *m_clBufferBatches);
    CHECK_CL_ERROR(clErr, "Set pre-solve kernel param 3 (batches)");
    clErr = m_clKernelPreSolve.setArg(6, (cl_int)m_scene->m_enableFriction);
    CHECK_CL_ERROR(clErr, "Set pre-solve kernel param 6 (friction)");
    clErr = m_clKernelPreSolve.setArg(7, (cl_float)m_scene->m_dt);
    CHECK_CL_ERROR(clErr, "Set pre-solve kernel param 7 (dt)");

    cl_uint offset = 0;
    cl::NDRange local(m_clLocalSize);
    for(cl_uint batchSize : m_clBatchSizes)
    {
        clErr = m_clKernelPreSolve.setArg(4, offset);
        CHECK_CL_ERROR(clErr, "Set pre-solve kernel param 4 (batch offset)");
        clErr = m_clKernelPreSolve.setArg(5, batchSize);
        CHECK_CL_ERROR(clErr, "Set pre-solve kernel param 5 (batch size)");

        offset += batchSize;

        cl::NDRange global(CEIL_TO(batchSize,local[0]));

        clErr = m_clQueue.enqueueNDRangeKernel(m_clKernelPreSolve, cl::NullRange, global, local);
        CHECK_CL_ERROR(clErr, "Run pre-solve kernel");
    }
}

//--------------------------------------------------------------------------------------------------
void q3IslandSolverOcl::SolveContacts( )
{
    cl_int clErr;
    clErr = m_clKernelSolve.setArg(0, *m_clBufferVelocity);
    CHECK_CL_ERROR(clErr, "Set solve kernel param 0 (velocity)");
    clErr = m_clKernelSolve.setArg(1, *m_clBufferContactConstraintState);
    CHECK_CL_ERROR(clErr, "Set solve kernel param 1 (contact constraint state)");
    clErr = m_clKernelSolve.setArg(2, *m_clBufferContactState);
    CHECK_CL_ERROR(clErr, "Set solve kernel param 2 (contact state)");
    clErr = m_clKernelSolve.setArg(3, *m_clBufferBatches);
    CHECK_CL_ERROR(clErr, "Set solve kernel param 3 (batches)");
    clErr = m_clKernelSolve.setArg(6, (cl_int)m_scene->m_enableFriction);
    CHECK_CL_ERROR(clErr, "Set solve kernel param 6 (friction)");

    for(int i = 0; i < m_scene->m_iterations; ++i) {
        cl_uint offset = 0;
        cl::NDRange local(m_clLocalSize);
        for(cl_uint batchSize : m_clBatchSizes)
        {
            clErr = m_clKernelSolve.setArg(4, offset);
            CHECK_CL_ERROR(clErr, "Set solve kernel param 4 (batch offset)");
            clErr = m_clKernelSolve.setArg(5, batchSize);
            CHECK_CL_ERROR(clErr, "Set solve kernel param 5 (batch size)");

            offset += batchSize;

            cl::NDRange global(CEIL_TO(batchSize,local[0]));

            clErr = m_clQueue.enqueueNDRangeKernel(m_clKernelSolve, cl::NullRange, global, local);
            CHECK_CL_ERROR(clErr, "Run solve kernel");
        }
    }
}

//--------------------------------------------------------------------------------------------------
void q3IslandSolverOcl::Add( q3BodyRef *body )
{
    assert( m_bodyCount < m_bodyCapacity );

    body->body()->m_islandIndex = m_bodyCount;
    body->body()->m_islandId = m_islandId;

    m_bodies[ m_bodyCount++ ] = body;
}

//--------------------------------------------------------------------------------------------------
void q3IslandSolverOcl::Add( q3ContactConstraint *contact )
{
    assert( m_contactCount < m_contactCapacity );

    m_contactConstraints[ m_contactCount++ ] = contact;
    m_contactStateCount += contact->manifold.contactCount;
}

//--------------------------------------------------------------------------------------------------
void q3IslandSolverOcl::InitializeContacts() {
    unsigned contactStateCount = 0;
    q3Body *A, *B;
    for ( i32 i = 0; i < m_contactCount; ++i )
    {
        q3ContactConstraint *cc = m_contactConstraints[ i ];

        q3ContactConstraintStateOcl *c = m_contactConstraintStates + i;

        A = cc->bodyA->body();
        B = cc->bodyB->body();

        c->centerA = A->m_worldCenter;
        c->centerB = B->m_worldCenter;
        c->iA = A->m_invInertiaWorld;
        c->iB = B->m_invInertiaWorld;
        c->mA = A->m_invMass;
        c->mB = B->m_invMass;
        c->restitution = cc->restitution;
        c->friction = cc->friction;
        c->indexA = A->m_islandIndex;
        c->indexB = B->m_islandIndex;
        c->normal = cc->manifold.normal;
        c->tangentVectors[ 0 ] = cc->manifold.tangentVectors[ 0 ];
        c->tangentVectors[ 1 ] = cc->manifold.tangentVectors[ 1 ];
        c->contactCount = cc->manifold.contactCount;

        for ( i32 j = 0; j < c->contactCount; ++j )
        {
            q3ContactStateOcl *s = m_contactStates + contactStateCount;
            q3Contact *cp = cc->manifold.contacts + j;

            m_constraintIndicies[contactStateCount] = i;
            m_constraintPairs[contactStateCount] = {
                .indexA = c->indexA,
                .indexB = c->indexB,
            };

            s->constraintIndex = i;
            s->ra = cp->position - c->centerA;
            s->rb = cp->position - c->centerB;
            s->penetration = cp->penetration;
            s->normalImpulse = cp->normalImpulse;
            s->tangentImpulse[ 0 ] = cp->tangentImpulse[ 0 ];
            s->tangentImpulse[ 1 ] = cp->tangentImpulse[ 1 ];

            contactStateCount++;
        }
    }
}
