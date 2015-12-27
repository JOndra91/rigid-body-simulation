//--------------------------------------------------------------------------------------------------
/**
@file  q3ContactSolverOcl.h

@author Ondřej Janošík
@date   24/12/2015

*/
//--------------------------------------------------------------------------------------------------

#ifndef Q3CONTACTSOLVEROCL_H
#define Q3CONTACTSOLVEROCL_H

#include <CL/cl.hpp>
#include <openCLUtilities.hpp>

#include "../math/q3Math.h"
#include "../common/q3Settings.h"
#include "q3Island.h"
#include "q3ContactSolverCpu.h"

struct q3ContactPlan
{
    cl_uint contactConstraintStateIndex;
    cl_uint contactStateIndex;

    bool operator< (const q3ContactPlan &b) const
    {
        if(contactConstraintStateIndex < b.contactConstraintStateIndex)
        {
            return true;
        }

        return contactConstraintStateIndex == b.contactConstraintStateIndex
                && contactStateIndex < b.contactStateIndex;
    }
};

//--------------------------------------------------------------------------------------------------
// q3ContactSolverOcl
//--------------------------------------------------------------------------------------------------
struct q3ContactSolverOcl : q3ContactSolverCpu
{
    q3ContactSolverOcl(cl_device_type dev = CL_DEVICE_TYPE_DEFAULT);

    void ShutDown( void ) override;

    void PreSolve( r32 dt ) override;
    void Solve( void ) override;

    //-----------------------------------------------------------
    // Properties for OpenCL acceleration
    //-----------------------------------------------------------
    cl::Context m_clContext;
    cl::Program m_clProgram;
    cl::Kernel m_clKernel;
    cl::CommandQueue m_clQueue;
    cl::Buffer *m_clBufferVelocity;
    cl::Buffer *m_clBufferContactConstraintState;
    cl::Buffer *m_clBufferBatches;
    GarbageCollector m_clGC;

    std::vector<q3ContactPlan> m_clBatches;
    std::vector<cl_uint> m_clBatchSizes;
};

#endif // Q3CONTACTSOLVEROCL_H
