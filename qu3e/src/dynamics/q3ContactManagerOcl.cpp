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
    m_clProgram = buildProgramFromSource(*m_clContext, "./qu3e/kernels/q3ContactManagerOcl.cl");

    std::vector<cl::Kernel> kernels;
    m_clProgram.createKernels(&kernels);

    assert(kernels.size() == 1);

    for(auto &kernel : kernels) {
        std::string name = kernel.getInfo<CL_KERNEL_FUNCTION_NAME>();
        if(name == "testCollisions") {
            m_clKernelTestCollisions = kernel;
        }
    }

    std::vector<cl::Device> devices;

    m_clContext->getInfo(CL_CONTEXT_DEVICES, &devices);

    m_clLocalSize = m_clKernelTestCollisions.getWorkGroupInfo
        <CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE>(devices.front());

}

//--------------------------------------------------------------------------------------------------
void q3ContactManagerOcl::TestCollisions( void )
{
    if(m_contactCount == 0) {
        return;
    }

    cl_int clErr;
    Indicies *indexMemory, *indexPtr;
    q3Contact *contactMemory, *contactPtr;
    q3OldContactOcl *oldContactMemory, *oldContactPtr;
    q3ContactConstraintOcl *constraintMemory, *constraintPtr;
    q3ManifoldOcl *manifoldMemory, *manifoldPtr;
    q3ContactConstraint* constraint;

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
    assert_size(q3OldContactOcl, 16);

    cl::Buffer bodyBuffer(*m_clContext, CL_MEM_READ_ONLY
        , m_container->m_bodies.size() * sizeof(q3Body), NULL, &clErr
    );
    CHECK_CL_ERROR(clErr, "Buffer q3Body");

    cl::Buffer boxBuffer(*m_clContext, CL_MEM_READ_ONLY
        , m_container->m_boxes.size() * sizeof(q3Box), NULL, &clErr
    );
    CHECK_CL_ERROR(clErr, "Buffer q3Box");

    cl::Buffer contactBuffer(*m_clContext, CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR
        , m_contactCount * 8 * sizeof(q3Contact), NULL, &clErr
    );
    CHECK_CL_ERROR(clErr, "Buffer q3Contact");

    cl::Buffer oldContactBuffer(*m_clContext, CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR
        , m_contactCount * 8 * sizeof(q3OldContactOcl), NULL, &clErr
    );
    CHECK_CL_ERROR(clErr, "Buffer q3OldContactOcl");

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

    oldContactMemory =
        (q3OldContactOcl*) m_clQueue.enqueueMapBuffer
        ( oldContactBuffer, CL_TRUE, CL_MAP_WRITE, 0
        , m_contactCount * 8 * sizeof(q3OldContactOcl), NULL, NULL, &clErr);
    CHECK_CL_ERROR(clErr, "Map buffer q3OldContactOcl");

    constraintMemory =
        (q3ContactConstraintOcl*) m_clQueue.enqueueMapBuffer
        ( constraintBuffer, CL_TRUE, CL_MAP_WRITE, 0
        , m_contactCount * sizeof(q3ContactConstraintOcl), NULL, NULL, &clErr);
    CHECK_CL_ERROR(clErr, "Map buffer q3ContactConstraintOcl");

    manifoldMemory =
    (q3ManifoldOcl*) m_clQueue.enqueueMapBuffer
    ( manifoldBuffer, CL_TRUE, CL_MAP_WRITE, 0
        , m_contactCount * sizeof(q3ManifoldOcl), NULL, NULL, &clErr);
    CHECK_CL_ERROR(clErr, "Map buffer q3ManifoldOcl");

    indexPtr = indexMemory;
    constraintPtr = constraintMemory;
    manifoldPtr = manifoldMemory;
    int i = 0;
    constraint = m_contactList;
    while(constraint)
    {
        indexPtr->load(*constraint);
        oldContactPtr = oldContactMemory + (i * 8);
        int contactCount = constraint->manifold.contactCount;
        for(int j = 0; j < contactCount; ++j) {
            oldContactPtr[j].load(constraint->manifold.contacts[j]);
        }
        constraintPtr->load(*constraint);
        manifoldPtr->load(constraint->manifold);

        ++indexPtr;
        ++constraintPtr;
        ++manifoldPtr;
        ++i;
        constraint = constraint->next;
    }

    m_clQueue.enqueueUnmapMemObject(indexBuffer, indexMemory);
    m_clQueue.enqueueUnmapMemObject(oldContactBuffer, oldContactMemory);
    m_clQueue.enqueueUnmapMemObject(constraintBuffer, constraintMemory);
    m_clQueue.enqueueUnmapMemObject(manifoldBuffer, manifoldMemory);

    m_clQueue.enqueueWriteBuffer(bodyBuffer, CL_FALSE, 0
        , m_container->m_bodies.size() * sizeof(q3Body)
        , m_container->m_bodies.data()
    );
    m_clQueue.enqueueWriteBuffer(boxBuffer, CL_FALSE, 0
        , m_container->m_boxes.size() * sizeof(q3Box)
        , m_container->m_boxes.data()
    );
    m_clQueue.enqueueWriteBuffer(nodeBuffer, CL_FALSE, 0
        , m_broadphase.m_tree.m_count * sizeof(q3DynamicAABBTree::Node)
        , m_broadphase.m_tree.m_nodes
    );

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
    clErr = m_clKernelTestCollisions.setArg(argument++, oldContactBuffer);
    CHECK_CL_ERROR(clErr, "Set pre-solve kernel param %d (oldContactBuffer)");
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
    m_clQueue.enqueueNDRangeKernel(m_clKernelTestCollisions, cl::NullRange, global, local);

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

    // constraintPtr = constraintMemory;
    // manifoldPtr = manifoldMemory;
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
        contactPtr = contactMemory + (i * 8);
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

    m_clQueue.enqueueUnmapMemObject(constraintBuffer, constraintMemory);
    m_clQueue.enqueueUnmapMemObject(manifoldBuffer, manifoldMemory);
    m_clQueue.enqueueUnmapMemObject(contactBuffer, contactMemory);

    const char *spaces = "                    ";
    constraint = m_contactList;
    while( constraint ) {
        constraint->print(spaces + 20);
        constraint = constraint->next;
    }
}
