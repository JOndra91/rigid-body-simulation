//--------------------------------------------------------------------------------------------------
/**
@file    q3IslandSolverOcl.h

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

#ifndef Q3ISLANDSOLVEROCL_H
#define Q3ISLANDSOLVEROCL_H

#include <CL/cl.hpp>
#include <set>
#include <vector>
#include <openCLUtilities.hpp>

#include "../math/q3Math.h"
#include "../common/q3Settings.h"
#include "q3Body.h"
#include "q3Island.h"
#include "q3IslandSolver.h"

struct q3VelocityStateOcl
{
    q3Vec3 w;
    q3Vec3 v;
};

struct q3ContactStateOcl
{
    q3Vec3 ra;                   // Vector from C.O.M to contact position
    q3Vec3 rb;                   // Vector from C.O.M to contact position
    r32 tangentImpulse[ 2 ];     // Accumulated friction impulse
    r32 tangentMass[ 2 ];        // Tangent constraint mass
    r32 penetration;             // Depth of penetration from collision
    r32 normalImpulse;           // Accumulated normal impulse
    r32 bias;                    // Restitution + baumgarte
    r32 normalMass;              // Normal constraint mass
    u32 constraintIndex;
    char _padding[12];
};

struct q3ContactConstraintStateOcl
{
    q3Mat3 iA;  // inertia of body A
    q3Mat3 iB;  // inertia of body B
    q3Vec3 tangentVectors[ 2 ];    // Tangent vectors
    q3Vec3 normal;                // From A to B
    q3Vec3 centerA;
    q3Vec3 centerB;
    i32 contactCount;
    r32 mA; // mass of body A
    r32 mB; // mass of body B
    r32 restitution;
    r32 friction;
    i32 indexA;
    i32 indexB;
    char _padding[4];
};

//--------------------------------------------------------------------------------------------------
// q3IslandSolverOcl
//--------------------------------------------------------------------------------------------------
struct q3IslandSolverOcl : q3IslandSolver
{
    q3IslandSolverOcl(cl_device_type dev = CL_DEVICE_TYPE_DEFAULT);
    ~q3IslandSolverOcl();

    void Solve( q3Scene *scene ) override;

    void InitializeContacts( void );
    void PreSolveContacts( void );
    void SolveContacts( void );

    void Add( q3Body *body );
    void Add( q3ContactConstraint *contact );

    q3ContactConstraintStateOcl *m_contactConstraintStates;
    q3ContactStateOcl *m_contactStates;
    q3ContactConstraint **m_contactConstraints;
    q3VelocityStateOcl *m_velocities;
    q3Body **m_bodies;

    q3Scene *m_scene;

    i32 m_bodyCapacity;
    i32 m_contactCapacity;
    i32 m_contactCount;
    i32 m_contactStateCount;
    i32 m_bodyCount;


    //-----------------------------------------------------------
    // Properties for OpenCL acceleration
    //-----------------------------------------------------------
    cl::Context m_clContext;
    cl::Program m_clProgram;
    cl::Kernel m_clKernelPreSolve;
    cl::Kernel m_clKernelSolve;
    cl::CommandQueue m_clQueue;
    cl::Buffer *m_clBufferVelocity;
    cl::Buffer *m_clBufferContactState;
    cl::Buffer *m_clBufferContactConstraintState;
    cl::Buffer *m_clBufferBatches;
    GarbageCollector m_clGC;

    std::vector<cl_uint> m_clBatches;
    std::vector<cl_uint> m_clBatchSizes;
};

#endif // Q3ISLANDSOLVEROCL_H
