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

#include "../math/q3Math.h"
#include "../common/q3Settings.h"
#include "q3ContactSolver.h"
#include "q3Island.h"


typedef q3ContactState q3ContactStateOcl;

struct q3BodyInfoOcl
{
    q3Vec3 center;
	q3Mat3 i;                   // Inverse inertia tensor
	r32 m;                      // Inverse mass
};

struct q3ContactConstraintStateOcl
{
	q3Vec3 tangentVectors[ 2 ];	// Tangent vectors
	q3Vec3 normal;              // From A to B
	r32 restitution;
	r32 friction;
};

struct q3ContactInfoOcl
{
    i32 contactStateIndex;
    i32 contactConstraintStateIndex;
    i32 vIndex;                 // Index to velocity and bodyInfo arrays
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
	i32 m_contactCount;
	q3VelocityState *m_velocities;

	bool m_enableFriction;

    //-----------------------------------------------------------
    // Properties for OpenCL acceleration
    //-----------------------------------------------------------
    cl::Context m_clContext;
    cl::CommandQueue m_clQueue;
    cl::Buffer *m_clBufferVelocity;
    cl::Buffer *m_clBufferBodyInfo;
    cl::Buffer *m_clBufferContactInfo;
    cl::Buffer *m_clBufferContactState;
    cl::Buffer *m_clBufferContactConstraintState;
    GarbageCollector m_clGC;

    q3ContactConstraintStateOcl *m_clContactConstraints;
    q3ContactStateOcl *m_clContactStates;
    q3BodyInfoOcl *m_clBodyInfos;
    q3ContactInfoOcl *m_clContactInfos;

    i32 m_clContactCount;
    i32 m_clContactStateCount;
};

#endif // Q3CONTACTSOLVEROCL_H
