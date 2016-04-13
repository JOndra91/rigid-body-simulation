//--------------------------------------------------------------------------------------------------
/**
@file    q3Island.h

@author    Randy Gaul
@date    10/10/2014

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

#ifndef Q3ISLAND_H
#define Q3ISLAND_H

#include "../math/q3Math.h"
#include "../common/q3Geometry.h"
#include "../common/q3Settings.h"
#include "q3ContactSolver.h"

//--------------------------------------------------------------------------------------------------
// q3Island
//--------------------------------------------------------------------------------------------------
class q3BroadPhase;
class q3Body;
struct q3ContactConstraint;

struct q3VelocityState
{
    q3Vec3 w;
    q3Vec3 v;
};

struct q3ContactState
{
    q3Vec3 ra;                    // Vector from C.O.M to contact position
    q3Vec3 rb;                    // Vector from C.O.M to contact position
    r32 penetration;            // Depth of penetration from collision
    r32 normalImpulse;            // Accumulated normal impulse
    r32 tangentImpulse[ 2 ];    // Accumulated friction impulse
    r32 bias;                    // Restitution + baumgarte
    r32 normalMass;                // Normal constraint mass
    r32 tangentMass[ 2 ];        // Tangent constraint mass
};

struct q3ContactConstraintState
{
    q3ContactState contacts[ 8 ];
    q3Vec3 tangentVectors[ 2 ];    // Tangent vectors
    q3Vec3 normal;                // From A to B
    q3Vec3 centerA;
    q3Vec3 centerB;
    q3Mat3 iA;
    q3Mat3 iB;
    i32 contactCount;
    r32 mA;
    r32 mB;
    r32 restitution;
    r32 friction;
    i32 indexA;
    i32 indexB;
#ifdef WITH_OCL
    i32 _padding;
#endif // WITH_OCL
};

enum class q3OpenCLDevice {
        NONE,
#ifdef WITH_OCL
        CPU,
        GPU,
#endif // WITH_OCL
    };

struct q3Island
{
#ifdef WITH_OCL
    q3Island(q3OpenCLDevice dev = q3OpenCLDevice::NONE);

    q3OpenCLDevice m_dev;
    inline bool UsesOpenCL() {
        return m_dev != q3OpenCLDevice::NONE;
    }
#else
    q3Island();

    inline bool UsesOpenCL() {
        return false;
    }
#endif // WITH_OCL
    ~q3Island();

    void Solve( );
    void Add( q3Body *body );
    void Add( q3ContactConstraint *contact );
    void Initialize( );

    q3ContactSolver *m_solver;

    q3Body **m_bodies;
    q3VelocityState *m_velocities;
    i32 m_bodyCapacity;
    i32 m_bodyCount;

    q3ContactConstraint **m_contacts;
    q3ContactConstraintState *m_contactStates;
    i32 m_contactCount;
    i32 m_contactCapacity;

    r32 m_dt;
    q3Vec3 m_gravity;
    i32 m_iterations;

    bool m_allowSleep;
    bool m_enableFriction;
};

#endif // Q3ISLAND_H
