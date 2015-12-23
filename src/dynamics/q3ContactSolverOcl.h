//--------------------------------------------------------------------------------------------------
/**
@file	q3ContactSolverOcl.h

@author	Randy Gaul
@date	10/10/2014

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

#ifndef Q3CONTACTSOLVEROCL_H
#define Q3CONTACTSOLVEROCL_H

#include <CL/cl.hpp>
#include <openCLUtilities.hpp>

#include "../math/q3Math.h"
#include "../common/q3Settings.h"
#include "q3ContactSolver.h"
#include "q3Island.h"

typedef q3ContactState q3ContactStateOcl;

struct cl_vec3
{
    cl_float3 cl;

    cl_vec3& operator=( const q3Vec3& vec)
    {
        cl.s[0] = vec.v[0];
        cl.s[1] = vec.v[1];
        cl.s[2] = vec.v[2];
    }
};

struct cl_vec3x3
{
    cl_vec3 ex;
	cl_vec3 ey;
	cl_vec3 ez;

    cl_vec3x3& operator=( const q3Mat3& mat )
    {
        ex = mat.ex;
        ey = mat.ey;
        ez = mat.ez;
    }
};

struct q3BodyInfoOcl
{
    cl_vec3 center;
	cl_vec3x3 i;      // Inverse inertia tensor
	cl_float m;         // Inverse mass
};

struct q3ContactConstraintStateOcl
{
	cl_vec3 tangentVectors[ 2 ];	// Tangent vectors
	cl_vec3 normal;               // From A to B
	cl_float restitution;
	cl_float friction;
};

struct q3ContactInfoOcl
{
    cl_uint contactStateIndex;
    cl_uint contactConstraintStateIndex;
    cl_uint vIndex;                 // Index to velocity and bodyInfo arrays
    // It's possible to determine according to index (starting with 0: even - A, odd - B)
    //i32 isA;                   // Whether it's A or B, so we can use correct normal
};

//--------------------------------------------------------------------------------------------------
// q3ContactSolverOcl
//--------------------------------------------------------------------------------------------------
struct q3ContactSolverOcl : q3ContactSolver
{
	q3ContactSolverOcl();

	void Initialize( q3Island *island ) override;
	void ShutDown( void ) override;

	void PreSolve( r32 dt ) override;
	void Solve( void ) override;

	q3Island *m_island;
	q3ContactConstraintState *m_contacts;
	u32 m_contactCount;
	q3VelocityState *m_velocities;

	bool m_enableFriction;

    //-----------------------------------------------------------
    // Properties for OpenCL acceleration
    //-----------------------------------------------------------
    cl::Context m_clContext;
    cl::Program m_clProgram;
    cl::Kernel m_clKernel;
    cl::CommandQueue m_clQueue;
    cl::Buffer *m_clBufferVelocity;
    cl::Buffer *m_clBufferBodyInfo;
    cl::Buffer *m_clBufferContactInfo;
    cl::Buffer *m_clBufferContactState;
    cl::Buffer *m_clBufferContactConstraintState;
    cl::Buffer *m_clBufferBatches;
    GarbageCollector m_clGC;

    std::vector<cl_uint> m_clBatches;
    std::vector<cl_uint> m_clBatchSizes;

    q3ContactConstraintStateOcl *m_clContactConstraints;
    q3ContactStateOcl *m_clContactStates;
    q3BodyInfoOcl *m_clBodyInfos;
    q3ContactInfoOcl *m_clContactInfos;

    u32 m_clContactCount;
    u32 m_clContactStateCount;
};

#endif // Q3CONTACTSOLVEROCL_H
