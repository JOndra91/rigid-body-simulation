//--------------------------------------------------------------------------------------------------
/**
@file  q3ContactSolverOcl.cpp

@author Ondřej Janošík
@date   24/12/2015

*/
//--------------------------------------------------------------------------------------------------

#include <CL/cl.hpp>
#include <openCLUtilities.hpp>
#include <iostream>
#include <vector>
#include <set>

#include "q3ContactSolverOcl.h"

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
void q3ContactSolverOcl::ShutDown( void )
{
    if(PASSED_ACC_THRESHOLD)
    {
        clErr = m_clQueue.enqueueReadBuffer(*m_clBufferVelocity, true, 0, sizeof(q3VelocityState) * m_island->m_bodyCount, m_velocities);
        CHECK_CL_ERROR("Read buffer q3VelocityState");
        clErr = m_clQueue.enqueueReadBuffer(*m_clBufferContactConstraintState, true, 0, sizeof(q3ContactConstraintState) * m_contactCount, m_contacts);
        CHECK_CL_ERROR("Read buffer q3ContactConstraintState");
    }

    q3ContactSolverCpu::ShutDown();

    m_clGC.deleteAllMemObjects();

    m_clBatches.clear();
    m_clBatchSizes.clear();
}

//--------------------------------------------------------------------------------------------------
void q3ContactSolverOcl::PreSolve( r32 dt )
{
    q3ContactSolverCpu::PreSolve(dt);

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

        // clErr = m_clQueue.finish();
        // CHECK_CL_ERROR("Finish batches");
    }
    else
    {
        q3ContactSolverCpu::Solve();
    }
}
