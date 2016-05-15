//--------------------------------------------------------------------------------------------------
/**
@file    q3ContactManagerOcl.h

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

#ifndef Q3CONTACTMANAGER_OCL_H
#define Q3CONTACTMANAGER_OCL_H

#include "q3ContactManager.h"
#include "q3Contact.h"
#include "../scene/q3Container.h"
#include <CL/cl.hpp>
#include "../common/q3OpenCL.h"

struct Indicies
{
    u32 boxA;
    u32 boxB;
    u32 bodyA;
    u32 bodyB;

    inline void load(const q3ContactConstraint &other) {
        boxA = other.A->m_boxIndex;
        boxB = other.B->m_boxIndex;
        bodyA = other.bodyA->m_bodyIndex;
        bodyB = other.bodyB->m_bodyIndex;
    };
} ALIGNED;

struct q3ManifoldOcl
{
    q3Vec3 tangentVectors[ 2 ];    // Tangent vectors
    q3Vec3 normal;                // From A to B
    i32 contactCount;
    u32 sensor;

    inline void load(const q3Manifold &other) {
        // contactCount = 0;
        tangentVectors[0] = other.tangentVectors[0];
        tangentVectors[1] = other.tangentVectors[1];

        normal = other.normal;
        sensor = other.sensor;
    }

    inline void update(q3Manifold &other) {
        other.tangentVectors[0] = tangentVectors[0];
        other.tangentVectors[1] = tangentVectors[1];

        other.contactCount = contactCount;

        other.normal = normal;
        other.sensor = sensor;
    }

} ALIGNED;

struct q3ContactConstraintOcl
{
    r32 friction;
    r32 restitution;
    i32 m_flags;

    inline void load(const q3ContactConstraint &other) {
        friction = other.friction;
        restitution = other.restitution;
        m_flags = other.m_flags;
    };

    inline void update(q3ContactConstraint &other) {
        other.friction = friction;
        other.restitution = restitution;
        other.m_flags = m_flags;
    };
} ALIGNED;

struct q3OldContactOcl {
    r32 tangentImpulse[ 2 ];

    inline void load(const q3Contact &other) {
        tangentImpulse[0] = other.tangentImpulse[0];
        tangentImpulse[1] = other.tangentImpulse[1];
    };
} ALIGNED;

//--------------------------------------------------------------------------------------------------
// q3ContactManagerOcl
//--------------------------------------------------------------------------------------------------

class q3ContactManagerOcl : public q3ContactManager
{
public:
    q3ContactManagerOcl( q3Stack* stack, q3Container *container, cl::Context *ctx);

    // Remove contacts without broadphase overlap
    // Solves contact manifolds
    void TestCollisions( void ) override;

private:
    q3Container *m_container;
    cl::Context *m_clContext;
    cl::CommandQueue m_clQueue;
    cl::Program m_clProgram;
    cl::Kernel m_clKernelTestCollisions;
    cl_uint m_clLocalSize;
};

#endif // Q3CONTACTMANAGER_OCL_H
