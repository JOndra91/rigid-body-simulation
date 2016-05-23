//--------------------------------------------------------------------------------------------------
/**
@file    q3ContactManagerOcl.cpp

@author  Ondřej Janošík
@date    13/5/2016

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

#include "q3ContactManagerOcl.h"
#include "../collision/q3Box.h"
#include "q3Body.h"
#include "q3Contact.h"
#include "../scene/q3Scene.h"
#include <openCLUtilities.hpp>
#include "../debug/q3Timers.h"

#ifdef NO_KERNEL_SOURCE
std::string managerKernelSource = "";
#else
std::string managerKernelSource =
#include "q3ContactManagerOcl.cl.str"
;
#endif // NO_KERNEL_SOURCE

//--------------------------------------------------------------------------------------------------
// q3ContactManagerOcl
//--------------------------------------------------------------------------------------------------
q3ContactManagerOcl::q3ContactManagerOcl( q3Stack* stack,  q3Container *container, cl::Context *ctx )
    : q3ContactManager(stack)
    , m_container(container)
    , m_clContext(ctx)
{
    m_clQueue = cl::CommandQueue(*m_clContext);

    // m_clProgram = buildProgramFromSourceString(*m_clContext, managerKernelSource);
    m_clProgram = buildProgramFromSource(*m_clContext, "./qu3e/kernels/q3ContactManagerOcl.cl", "-I./qu3e/kernels");

    std::vector<cl::Device> devices;
    m_clContext->getInfo(CL_CONTEXT_DEVICES, &devices);

    // std::string log = m_clProgram.getBuildInfo<CL_PROGRAM_BUILD_LOG>(devices.front());
    //
    // printf("Log: \n%s\n", log.c_str());

    std::vector<cl::Kernel> kernels;
    m_clProgram.createKernels(&kernels);

    assert(kernels.size() == 1);

    for(auto &kernel : kernels) {
        std::string name = kernel.getInfo<CL_KERNEL_FUNCTION_NAME>();
        if(name == "testCollisions") {
            m_clKernelTestCollisions = kernel;
        }
    }


    m_clLocalSize = m_clKernelTestCollisions.getWorkGroupInfo
        <CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE>(devices.front());

    assert_size(i32, 4);
    assert_size(r32, 4);
    assert_size(u32, 4);
    assert_size(q3Vec3, 16);
    assert_size(q3Mat3, 48);
    assert_size(Indicies, 16);
    assert_size(q3AABB, 32);
    assert_size(q3DynamicAABBTree::Node, 48); // aabbNode
    assert_size(q3Transform, 64);
    assert_size(q3Box, 112);
    assert_size(q3Body, 320);
    assert_size(q3FeaturePair, 4);
    assert_size(q3Contact, 64);
    assert_size(q3ManifoldOcl, 64);
    assert_size(q3ContactConstraintOcl, 16);
    assert_size(q3ClipVertex, 32);

    // printf("offsetof(Indicies.boxA) = %lu\n", offsetof(Indicies, boxA));
    // printf("offsetof(Indicies.boxB) = %lu\n", offsetof(Indicies, boxB));
    // printf("offsetof(Indicies.bodyA) = %lu\n", offsetof(Indicies, bodyA));
    // printf("offsetof(Indicies.bodyB) = %lu\n", offsetof(Indicies, bodyB));
    //
    // printf("offsetof(q3AABB.min) = %lu\n", offsetof(q3AABB, min));
    // printf("offsetof(q3AABB.max) = %lu\n", offsetof(q3AABB, max));
    //
    // printf("offsetof(aabbNode.aabb) = %lu\n", offsetof(q3DynamicAABBTree::Node, aabb));
    // printf("offsetof(aabbNode.height) = %lu\n", offsetof(q3DynamicAABBTree::Node, height));
    //
    // printf("offsetof(q3ContactConstraintOcl.friction) = %lu\n", offsetof(q3ContactConstraintOcl, friction));
    // printf("offsetof(q3ContactConstraintOcl.restitution) = %lu\n", offsetof(q3ContactConstraintOcl, restitution));
    // printf("offsetof(q3ContactConstraintOcl.m_flags) = %lu\n", offsetof(q3ContactConstraintOcl, m_flags));
    //
    // printf("offsetof(q3ClipVertex.v) = %lu\n", offsetof(q3ClipVertex, v));
    // printf("offsetof(q3ClipVertex.f) = %lu\n", offsetof(q3ClipVertex, f));
    //
    // printf("offsetof(q3ManifoldOcl.tangentVectors[0]) = %lu\n", offsetof(q3ManifoldOcl, tangentVectors[0]));
    // printf("offsetof(q3ManifoldOcl.tangentVectors[1]) = %lu\n", offsetof(q3ManifoldOcl, tangentVectors[1]));
    // printf("offsetof(q3ManifoldOcl.normal) = %lu\n", offsetof(q3ManifoldOcl, normal));
    // printf("offsetof(q3ManifoldOcl.contactCount) = %lu\n", offsetof(q3ManifoldOcl, contactCount));
    // printf("offsetof(q3ManifoldOcl.sensor) = %lu\n", offsetof(q3ManifoldOcl, sensor));
    //
    // printf("offsetof(q3FeaturePairOcl.key) = %lu\n", offsetof(q3FeaturePair, key));
    // printf("offsetof(q3FeaturePairOcl.inR) = %lu\n", offsetof(q3FeaturePair, inR));
    // printf("offsetof(q3FeaturePairOcl.outR) = %lu\n", offsetof(q3FeaturePair, outR));
    // printf("offsetof(q3FeaturePairOcl.inI) = %lu\n", offsetof(q3FeaturePair, inI));
    // printf("offsetof(q3FeaturePairOcl.outI) = %lu\n", offsetof(q3FeaturePair, outI));
    //
    // printf("offsetof(q3ContactOcl.position) = %lu\n", offsetof(q3Contact, position));
    // printf("offsetof(q3ContactOcl.penetration) = %lu\n", offsetof(q3Contact, penetration));
    // printf("offsetof(q3ContactOcl.normalImpulse) = %lu\n", offsetof(q3Contact, normalImpulse));
    // printf("offsetof(q3ContactOcl.tangentImpulse[0]) = %lu\n", offsetof(q3Contact, tangentImpulse[0]));
    // printf("offsetof(q3ContactOcl.tangentImpulse[1]) = %lu\n", offsetof(q3Contact, tangentImpulse[1]));
    // printf("offsetof(q3ContactOcl.bias) = %lu\n", offsetof(q3Contact, bias));
    // printf("offsetof(q3ContactOcl.normalMass) = %lu\n", offsetof(q3Contact, normalMass));
    // printf("offsetof(q3ContactOcl.tangentMass[0]) = %lu\n", offsetof(q3Contact, tangentMass[0]));
    // printf("offsetof(q3ContactOcl.tangentMass[1]) = %lu\n", offsetof(q3Contact, tangentMass[1]));
    // printf("offsetof(q3ContactOcl.fp) = %lu\n", offsetof(q3Contact, fp));
    // printf("offsetof(q3ContactOcl.warmStarted) = %lu\n", offsetof(q3Contact, warmStarted));
    //
    // printf("offsetof(q3Body.m_invInertiaModel) = %lu\n", offsetof(q3Body, m_invInertiaModel));
    // printf("offsetof(q3Body.m_invInertiaWorld) = %lu\n", offsetof(q3Body, m_invInertiaWorld));
    // printf("offsetof(q3Body.m_mass) = %lu\n", offsetof(q3Body, m_mass));
    // printf("offsetof(q3Body.m_invMass) = %lu\n", offsetof(q3Body, m_invMass));
    // printf("offsetof(q3Body.m_linearVelocity) = %lu\n", offsetof(q3Body, m_linearVelocity));
    // printf("offsetof(q3Body.m_angularVelocity) = %lu\n", offsetof(q3Body, m_angularVelocity));
    // printf("offsetof(q3Body.m_force) = %lu\n", offsetof(q3Body, m_force));
    // printf("offsetof(q3Body.m_torque) = %lu\n", offsetof(q3Body, m_torque));
    // printf("offsetof(q3Body.m_tx) = %lu\n", offsetof(q3Body, m_tx));
    // printf("offsetof(q3Body.m_q) = %lu\n", offsetof(q3Body, m_q));
    // printf("offsetof(q3Body.m_localCenter) = %lu\n", offsetof(q3Body, m_localCenter));
    // printf("offsetof(q3Body.m_worldCenter) = %lu\n", offsetof(q3Body, m_worldCenter));
    // printf("offsetof(q3Body.m_sleepTime) = %lu\n", offsetof(q3Body, m_sleepTime));
    // printf("offsetof(q3Body.m_gravityScale) = %lu\n", offsetof(q3Body, m_gravityScale));
    // printf("offsetof(q3Body.m_layers) = %lu\n", offsetof(q3Body, m_layers));
    // printf("offsetof(q3Body.m_flags) = %lu\n", offsetof(q3Body, m_flags));
    // printf("offsetof(q3Body.m_bodyIndex) = %lu\n", offsetof(q3Body, m_bodyIndex));
    // printf("offsetof(q3Body.m_islandIndex) = %lu\n", offsetof(q3Body, m_islandIndex));
    // printf("offsetof(q3Body.m_islandId) = %lu\n", offsetof(q3Body, m_islandId));
    //
    // printf("offsetof(q3Box.localTransform) = %lu\n", offsetof(q3Box, local));
    // printf("offsetof(q3Box.e) = %lu\n", offsetof(q3Box, e));
    // printf("offsetof(q3Box.friction) = %lu\n", offsetof(q3Box, friction));
    // printf("offsetof(q3Box.restitution) = %lu\n", offsetof(q3Box, restitution));
    // printf("offsetof(q3Box.density) = %lu\n", offsetof(q3Box, density));
    // printf("offsetof(q3Box.broadPhaseIndex) = %lu\n", offsetof(q3Box, broadPhaseIndex));
    // printf("offsetof(q3Box.sensor) = %lu\n", offsetof(q3Box, sensor));
    // printf("offsetof(q3Box.m_boxIndex) = %lu\n", offsetof(q3Box, m_boxIndex));
    // printf("offsetof(q3Box.m_bodyIndex) = %lu\n", offsetof(q3Box, m_bodyIndex));
    //
    // printf("offsetof(q3Transform.rotation) = %lu\n", offsetof(q3Transform, rotation));
    // printf("offsetof(q3Transform.position) = %lu\n", offsetof(q3Transform, position));
}

//--------------------------------------------------------------------------------------------------
void q3ContactManagerOcl::TestCollisions( void )
{
    if(m_contactCount == 0) {
        return;
    }

    int i;
    cl_int clErr;
    Indicies *indexMemory;
    q3Contact *contactMemory;
    q3ContactConstraintOcl *constraintMemory;
    q3ManifoldOcl *manifoldMemory;
    q3ContactConstraint* constraint;

    q3TimerStart("manager-ocl");

    cl::Buffer bodyBuffer(*m_clContext, CL_MEM_READ_ONLY
        , m_container->m_bodies.size() * sizeof(q3Body), NULL, &clErr
    );
    CHECK_CL_ERROR(clErr, "Buffer q3Body");

    cl::Buffer boxBuffer(*m_clContext, CL_MEM_READ_ONLY
        , m_container->m_boxes.size() * sizeof(q3Box), NULL, &clErr
    );
    CHECK_CL_ERROR(clErr, "Buffer q3Box");

    cl::Buffer contactBuffer(*m_clContext, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR
        , m_contactCount * 8 * sizeof(q3Contact), NULL, &clErr
    );
    CHECK_CL_ERROR(clErr, "Buffer q3Contact");

    cl::Buffer constraintBuffer(*m_clContext, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR
        , m_contactCount * sizeof(q3ContactConstraintOcl), NULL, &clErr
    );
    CHECK_CL_ERROR(clErr, "Buffer q3ContactConstraintOcl");

    cl::Buffer indexBuffer(*m_clContext, CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR
        , m_contactCount * sizeof(Indicies), NULL, &clErr
    );
    CHECK_CL_ERROR(clErr, "Buffer Indicies");

    cl::Buffer manifoldBuffer(*m_clContext, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR
        , m_contactCount * sizeof(q3ManifoldOcl), NULL, &clErr
    );
    CHECK_CL_ERROR(clErr, "Buffer q3ManifoldOcl");

    cl::Buffer nodeBuffer(*m_clContext, CL_MEM_READ_ONLY
        , m_broadphase.m_tree.m_count * sizeof(q3DynamicAABBTree::Node), NULL, &clErr
    );
    CHECK_CL_ERROR(clErr, "Buffer q3DynamicAABBTree::Node");

    indexMemory =
        (Indicies*) m_clQueue.enqueueMapBuffer
        ( indexBuffer, CL_TRUE, CL_MAP_WRITE, 0
        , m_contactCount * sizeof(Indicies), NULL, NULL, &clErr);
    CHECK_CL_ERROR(clErr, "Map buffer Indicies");
    // memset(indexMemory, 0, m_contactCount * sizeof(Indicies));

    contactMemory =
        (q3Contact*) m_clQueue.enqueueMapBuffer
        ( contactBuffer, CL_TRUE, CL_MAP_WRITE, 0
        , m_contactCount * 8 * sizeof(q3Contact), NULL, NULL, &clErr);
    CHECK_CL_ERROR(clErr, "Map buffer q3Contact");
    // memset(contactMemory, 0, m_contactCount * 8 * sizeof(q3Contact));

    constraintMemory =
        (q3ContactConstraintOcl*) m_clQueue.enqueueMapBuffer
        ( constraintBuffer, CL_TRUE, CL_MAP_WRITE, 0
        , m_contactCount * sizeof(q3ContactConstraintOcl), NULL, NULL, &clErr);
    CHECK_CL_ERROR(clErr, "Map buffer q3ContactConstraintOcl");
    // memset(constraintMemory, 0, m_contactCount * sizeof(q3ContactConstraintOcl));

    manifoldMemory =
        (q3ManifoldOcl*) m_clQueue.enqueueMapBuffer
        ( manifoldBuffer, CL_TRUE, CL_MAP_WRITE, 0
        , m_contactCount * sizeof(q3ManifoldOcl), NULL, NULL, &clErr);
    CHECK_CL_ERROR(clErr, "Map buffer q3ManifoldOcl");
    // memset(manifoldMemory, 0, m_contactCount * sizeof(q3ManifoldOcl));

    for(constraint = m_contactList, i = 0; constraint != NULL; constraint = constraint->next, ++i) {
        indexMemory[i].load(*constraint);
        q3Contact *contactPtr = contactMemory + (i * 8);
        int contactCount = constraint->manifold.contactCount;
        for(int j = 0; j < contactCount; ++j) {
            contactPtr[j] = constraint->manifold.contacts[j];
        }
        constraintMemory[i].load(*constraint);
        manifoldMemory[i].load(constraint->manifold);
    }

    clErr = m_clQueue.enqueueUnmapMemObject(indexBuffer, indexMemory);
    CHECK_CL_ERROR(clErr, "Unmap buffer Indicies");
    clErr = m_clQueue.enqueueUnmapMemObject(contactBuffer, contactMemory);
    CHECK_CL_ERROR(clErr, "Unmap buffer q3Contact");
    clErr = m_clQueue.enqueueUnmapMemObject(constraintBuffer, constraintMemory);
    CHECK_CL_ERROR(clErr, "Unmap buffer q3ContactConstraintOcl");
    clErr = m_clQueue.enqueueUnmapMemObject(manifoldBuffer, manifoldMemory);
    CHECK_CL_ERROR(clErr, "Unmap buffer q3ManifoldOcl");

    clErr = m_clQueue.enqueueWriteBuffer(bodyBuffer, CL_TRUE, 0
        , m_container->m_bodies.size() * sizeof(q3Body)
        , m_container->m_bodies.data()
    );
    CHECK_CL_ERROR(clErr, "Write buffer q3Body");
    clErr = m_clQueue.enqueueWriteBuffer(boxBuffer, CL_TRUE, 0
        , m_container->m_boxes.size() * sizeof(q3Box)
        , m_container->m_boxes.data()
    );
    CHECK_CL_ERROR(clErr, "Write buffer q3Box");
    clErr = m_clQueue.enqueueWriteBuffer(nodeBuffer, CL_TRUE, 0
        , m_broadphase.m_tree.m_count * sizeof(q3DynamicAABBTree::Node)
        , m_broadphase.m_tree.m_nodes
    );
    CHECK_CL_ERROR(clErr, "Write buffer q3DynamicAABBTree::Node");

    // Run kernel here

    cl_uint argument = 0;

    clErr = m_clKernelTestCollisions.setArg(argument++, indexBuffer);
    CHECK_CL_ERROR(clErr, "Set pre-solve kernel param %d (indexBuffer)");
    clErr = m_clKernelTestCollisions.setArg(argument++, constraintBuffer);
    CHECK_CL_ERROR(clErr, "Set pre-solve kernel param %d (constraintBuffer)");
    clErr = m_clKernelTestCollisions.setArg(argument++, manifoldBuffer);
    CHECK_CL_ERROR(clErr, "Set pre-solve kernel param %d (manifoldBuffer)");
    clErr = m_clKernelTestCollisions.setArg(argument++, contactBuffer);
    CHECK_CL_ERROR(clErr, "Set pre-solve kernel param %d (contactBuffer)");
    clErr = m_clKernelTestCollisions.setArg(argument++, bodyBuffer);
    CHECK_CL_ERROR(clErr, "Set pre-solve kernel param %d (bodyBuffer)");
    clErr = m_clKernelTestCollisions.setArg(argument++, boxBuffer);
    CHECK_CL_ERROR(clErr, "Set pre-solve kernel param %d (boxBuffer)");
    clErr = m_clKernelTestCollisions.setArg(argument++, nodeBuffer);
    CHECK_CL_ERROR(clErr, "Set pre-solve kernel param %d (nodeBuffer)");
    clErr = m_clKernelTestCollisions.setArg(argument++, m_contactCount);
    CHECK_CL_ERROR(clErr, "Set pre-solve kernel param %d (m_contactCount)");

    cl::NDRange local(m_clLocalSize);
    cl::NDRange global(CEIL_TO(m_contactCount,local[0]));
    clErr = m_clQueue.enqueueNDRangeKernel(m_clKernelTestCollisions, cl::NullRange, global, local);
    CHECK_CL_ERROR(clErr, "Run testCollisions kernel");
    m_clQueue.finish();
    printf("Kernel done\n");

    // Read back
    constraintMemory =
        (q3ContactConstraintOcl*) m_clQueue.enqueueMapBuffer
        ( constraintBuffer, CL_TRUE, CL_MAP_READ, 0
        , m_contactCount * sizeof(q3ContactConstraintOcl), NULL, NULL, &clErr);
    CHECK_CL_ERROR(clErr, "Map buffer q3ContactConstraintOcl");

    manifoldMemory =
    (q3ManifoldOcl*) m_clQueue.enqueueMapBuffer
    ( manifoldBuffer, CL_TRUE, CL_MAP_READ, 0
        , m_contactCount * sizeof(q3ManifoldOcl), NULL, NULL, &clErr);
    CHECK_CL_ERROR(clErr, "Map buffer q3ManifoldOcl");

    contactMemory =
    (q3Contact*) m_clQueue.enqueueMapBuffer
    ( contactBuffer, CL_TRUE, CL_MAP_READ, 0
        , m_contactCount * 8 * sizeof(q3Contact), NULL, NULL, &clErr);
    CHECK_CL_ERROR(clErr, "Map buffer q3Contact");

    q3TimerStop("manager-ocl");
    q3TimerPrint("manager-ocl", "  ManagerOCL");
    q3TimerStart("manager-cpu");

    for(constraint = m_contactList, i = 0; constraint; constraint = constraint->next, ++i)
    {
        constraintMemory[i].update(*constraint);

        if(constraint->m_flags & q3ContactConstraint::eRemove) {
            RemoveContact(constraint);
            // printf("Removed: %d\n", i);
            continue;
        }

        manifoldMemory[i].update(constraint->manifold);
        // printf("Contact count[%d]\n: %d", i, manifoldMemory[i].contactCount);

        int contactCount = constraint->manifold.contactCount;
        q3Contact *contactPtr = contactMemory + (i * 8);
        for(int j = 0; j < contactCount; ++j) {
            constraint->manifold.contacts[j] = contactPtr[j];
        }

        if ( m_contactListener )
        {
            if (
                constraint->m_flags & q3ContactConstraint::eColliding &&
                !(constraint->m_flags & q3ContactConstraint::eWasColliding)
                )
            {
                m_contactListener->BeginContact( constraint );
            }

            else if (
                !(constraint->m_flags & q3ContactConstraint::eColliding) &&
                constraint->m_flags & q3ContactConstraint::eWasColliding
                )
            {
                m_contactListener->EndContact( constraint );
            }
        }
    }

    m_clQueue.finish();
    q3TimerStop("manager-cpu");
    q3TimerPrint("manager-cpu", "  ManagerCPU");

    clErr = m_clQueue.enqueueUnmapMemObject(constraintBuffer, constraintMemory);
    CHECK_CL_ERROR(clErr, "Unmap buffer q3ContactConstraintOcl");
    clErr = m_clQueue.enqueueUnmapMemObject(manifoldBuffer, manifoldMemory);
    CHECK_CL_ERROR(clErr, "Unmap buffer q3ManifoldOcl");
    clErr = m_clQueue.enqueueUnmapMemObject(contactBuffer, contactMemory);
    CHECK_CL_ERROR(clErr, "Unmap buffer q3Contact");

    // const char *spaces = "                    ";
    // constraint = m_contactList;
    // while( constraint ) {
    //     constraint->print(spaces + 20);
    //     constraint = constraint->next;
    // }
}
