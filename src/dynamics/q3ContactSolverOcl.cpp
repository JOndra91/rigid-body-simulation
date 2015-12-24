//--------------------------------------------------------------------------------------------------
/**
@file  q3ContactSolverOcl.cpp

@author  Randy Gaul
@date  10/10/2014

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

#include <CL/cl.hpp>
#include <openCLUtilities.hpp>
#include <iostream>
#include <vector>
#include <set>
#include <iomanip>

#include "q3ContactSolver.h"
#include "q3ContactSolverOcl.h"
#include "q3Contact.h"
#include "q3Island.h"
#include "../common/q3Memory.h"
#include "q3Body.h"
#include "../common/q3Geometry.h"
#include "../common/q3Settings.h"

#define assert_size(type, size) assert(sizeof(type) == size)
//#define assert_size(type, size) do { std::cout << "sizeof(" << # type << ") = " << sizeof(type) << std::endl; assert(sizeof(type) == size); } while(0)

// Number of contacts needed for acceleration using OpenCL
#define ACCELERATION_THRESHOLD 64
#define PASSED_ACC_THRESHOLD (m_contactCount >= ACCELERATION_THRESHOLD)

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

//--------------------------------------------------------------------------------------------------
// q3ContactSolverOcl
//--------------------------------------------------------------------------------------------------
q3ContactSolverOcl::q3ContactSolverOcl()
{

    assert_size(i32, sizeof(cl_int));
    assert_size(r32, sizeof(cl_float));
    assert_size(q3Vec3, sizeof(cl_float3));
    assert_size(q3Mat3, sizeof(cl_float3) * 3);
    assert_size(q3VelocityState, sizeof(cl_float3) * 2);
    assert_size(q3ContactState, 64);
    assert_size(q3ContactConstraintState, 720);
    assert_size(q3ContactPlan, sizeof(cl_int) * 2);


    m_clContext = createCLContext(CL_DEVICE_TYPE_CPU);
    m_clQueue = cl::CommandQueue(m_clContext);

    m_clProgram = buildProgramFromSource(m_clContext, "q3ContactSolverOcl.cl");

    std::vector<cl::Kernel> kernels;
    m_clProgram.createKernels(&kernels);

    // There is only one kernel
    m_clKernel = kernels.front();
}
//--------------------------------------------------------------------------------------------------
void q3ContactSolverOcl::Initialize( q3Island *island )
{
    m_island = island;
    m_contactCount = island->m_contactCount;
    m_contacts = island->m_contactStates;
    m_velocities = m_island->m_velocities;
    m_enableFriction = island->m_enableFriction;

}

//--------------------------------------------------------------------------------------------------
void q3ContactSolverOcl::ShutDown( void )
{
    if(PASSED_ACC_THRESHOLD)
    {
        clErr = m_clQueue.enqueueReadBuffer(*m_clBufferVelocity, true, 0, sizeof(q3VelocityState) * m_island->m_bodyCount, m_velocities);
        CHECK_CL_ERROR("Read buffer q3VelocityState");
        clErr = m_clQueue.enqueueReadBuffer(*m_clBufferContactConstraintState, true, 0, sizeof(q3ContactConstraintState) * m_contactCount, m_contacts);
        CHECK_CL_ERROR("Read buffer q3ContactConstraintState");
    }

    for ( i32 i = 0; i < m_contactCount; ++i )
    {
        q3ContactConstraintState *c = m_contacts + i;
        q3ContactConstraint *cc = m_island->m_contacts[ i ];

        for ( i32 j = 0; j < c->contactCount; ++j )
        {
            q3Contact *oc = cc->manifold.contacts + j;
            q3ContactState *cs = c->contacts + j;
            oc->normalImpulse = cs->normalImpulse;
            oc->tangentImpulse[ 0 ] = cs->tangentImpulse[ 0 ];
            oc->tangentImpulse[ 1 ] = cs->tangentImpulse[ 1 ];
        }
    }


    m_clGC.deleteAllMemObjects();

    m_clBatches.clear();
    m_clBatchSizes.clear();
}

//--------------------------------------------------------------------------------------------------
void q3ContactSolverOcl::PreSolve( r32 dt )
{
  for ( i32 i = 0; i < m_contactCount; ++i )
  {
    q3ContactConstraintState *cs = m_contacts + i;

    q3Vec3 vA = m_velocities[ cs->indexA ].v;
    q3Vec3 wA = m_velocities[ cs->indexA ].w;
    q3Vec3 vB = m_velocities[ cs->indexB ].v;
    q3Vec3 wB = m_velocities[ cs->indexB ].w;

    for ( i32 j = 0; j < cs->contactCount; ++j )
    {
      q3ContactState *c = cs->contacts + j;

      // Precalculate JM^-1JT for contact and friction constraints
      q3Vec3 raCn = q3Cross( c->ra, cs->normal );
      q3Vec3 rbCn = q3Cross( c->rb, cs->normal );
      r32 nm = cs->mA + cs->mB;
      r32 tm[ 2 ];
      tm[ 0 ] = nm;
      tm[ 1 ] = nm;

      nm += q3Dot( raCn, cs->iA * raCn ) + q3Dot( rbCn, cs->iB * rbCn );
      c->normalMass = q3Invert( nm );

      for ( i32 i = 0; i < 2; ++i )
      {
        q3Vec3 raCt = q3Cross( cs->tangentVectors[ i ], c->ra );
        q3Vec3 rbCt = q3Cross( cs->tangentVectors[ i ], c->rb );
        tm[ i ] += q3Dot( raCt, cs->iA * raCt ) + q3Dot( rbCt, cs->iB * rbCt );
        c->tangentMass[ i ] = q3Invert( tm[ i ] );
      }

      // Precalculate bias factor
      c->bias = -Q3_BAUMGARTE * (r32( 1.0 ) / dt) * q3Min( r32( 0.0 ), c->penetration + Q3_PENETRATION_SLOP );

      // Warm start contact
      q3Vec3 P = cs->normal * c->normalImpulse;

      if ( m_enableFriction )
      {
        P += cs->tangentVectors[ 0 ] * c->tangentImpulse[ 0 ];
        P += cs->tangentVectors[ 1 ] * c->tangentImpulse[ 1 ];
      }

      vA -= P * cs->mA;
      wA -= cs->iA * q3Cross( c->ra, P );

      vB += P * cs->mB;
      wB += cs->iB * q3Cross( c->rb, P );

      // Add in restitution bias
      r32 dv = q3Dot( vB + q3Cross( wB, c->rb ) - vA - q3Cross( wA, c->ra ), cs->normal );

      if ( dv < -r32( 1.0 ) )
        c->bias += -(cs->restitution) * dv;
    }

    m_velocities[ cs->indexA ].v = vA;
    m_velocities[ cs->indexA ].w = wA;
    m_velocities[ cs->indexB ].v = vB;
    m_velocities[ cs->indexB ].w = wB;
  }

  if(PASSED_ACC_THRESHOLD)
  {
    m_clBufferVelocity = new cl::Buffer(m_clContext, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, sizeof(q3VelocityState) * m_island->m_bodyCount, m_velocities, &clErr);
    CHECK_CL_ERROR("Buffer q3VelocityState");
    m_clBufferContactConstraintState = new cl::Buffer(m_clContext, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, sizeof(q3ContactConstraintState) * m_contactCount, m_contacts, &clErr);
    CHECK_CL_ERROR("Buffer q3ContactConstraintStateOcl");

    m_clGC.addMemObject(m_clBufferVelocity);
    m_clGC.addMemObject(m_clBufferContactConstraintState);

    std::vector<unsigned> bodyAllocationTable(m_island->m_bodyCount, 0);

    unsigned contactCountTotal = 0;
    std::set<q3ContactPlan> contactsToPlan;
    q3ContactPlan plan;
    for(cl_uint i = 0; i < m_contactCount; i++)
    {
        contactCountTotal += m_contacts[i].contactCount;

        plan.contactConstraintStateIndex = i;
        for(cl_uint j = 0; j < m_contacts[i].contactCount; j++)
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
        q3ContactConstraintState *cc;
        while(it != end) {
            cc = m_contacts + it->contactConstraintStateIndex;

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

    //            std::cout << "Batch size:" << m_clBatches.size() - batchOffset << std::endl;
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
void q3ContactSolverOcl::Solve( )
{
    if(PASSED_ACC_THRESHOLD) {

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

        clErr = m_clQueue.finish();
        CHECK_CL_ERROR("Finish batches");
    }
    else
    {

        for ( i32 i = 0; i < m_contactCount; ++i )
        {
            q3ContactConstraintState *cs = m_contacts + i;

            q3Vec3 vA = m_velocities[ cs->indexA ].v;
            q3Vec3 wA = m_velocities[ cs->indexA ].w;
            q3Vec3 vB = m_velocities[ cs->indexB ].v;
            q3Vec3 wB = m_velocities[ cs->indexB ].w;

            for ( i32 j = 0; j < cs->contactCount; ++j )
            {
                q3ContactState *c = cs->contacts + j;

                // relative velocity at contact
                q3Vec3 dv = vB + q3Cross( wB, c->rb ) - vA - q3Cross( wA, c->ra );

                // Friction
                if ( m_enableFriction )
                {
                    for ( i32 i = 0; i < 2; ++i )
                    {
                        r32 lambda = -q3Dot( dv, cs->tangentVectors[ i ] ) * c->tangentMass[ i ];

                        // Calculate frictional impulse
                        r32 maxLambda = cs->friction * c->normalImpulse;

                        // Clamp frictional impulse
                        r32 oldPT = c->tangentImpulse[ i ];
                        c->tangentImpulse[ i ] = q3Clamp( -maxLambda, maxLambda, oldPT + lambda );
                        lambda = c->tangentImpulse[ i ] - oldPT;

                        // Apply friction impulse
                        q3Vec3 impulse = cs->tangentVectors[ i ] * lambda;
                        vA -= impulse * cs->mA;
                        wA -= cs->iA * q3Cross( c->ra, impulse );

                        vB += impulse * cs->mB;
                        wB += cs->iB * q3Cross( c->rb, impulse );
                    }
                }

                // Normal
                {
                    dv = vB + q3Cross( wB, c->rb ) - vA - q3Cross( wA, c->ra );

                    // Normal impulse
                    r32 vn = q3Dot( dv, cs->normal );

                    // Factor in positional bias to calculate impulse scalar j
                    r32 lambda = c->normalMass * (-vn + c->bias);

                    // Clamp impulse
                    r32 tempPN = c->normalImpulse;
                    c->normalImpulse = q3Max( tempPN + lambda, r32( 0.0 ) );
                    lambda = c->normalImpulse - tempPN;

                    // Apply impulse
                    q3Vec3 impulse = cs->normal * lambda;
                    vA -= impulse * cs->mA;
                    wA -= cs->iA * q3Cross( c->ra, impulse );

                    vB += impulse * cs->mB;
                    wB += cs->iB * q3Cross( c->rb, impulse );
                }
            }

            m_velocities[ cs->indexA ].v = vA;
            m_velocities[ cs->indexA ].w = wA;
            m_velocities[ cs->indexB ].v = vB;
            m_velocities[ cs->indexB ].w = wB;
        }
    }
}
