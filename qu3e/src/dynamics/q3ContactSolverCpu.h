//--------------------------------------------------------------------------------------------------
/**
@file    q3ContactSolverCpu.h

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

#ifndef Q3CONTACTSOLVERCPU_H
#define Q3CONTACTSOLVERCPU_H

#include "../math/q3Math.h"
#include "../common/q3Settings.h"
#include "q3Island.h"

//--------------------------------------------------------------------------------------------------
// q3ContactSolverCpu
//--------------------------------------------------------------------------------------------------
struct q3ContactSolverCpu
{
    void Initialize( q3Island *island );
    void ShutDown( void );

    void PreSolve( r32 dt );
    void Solve( void );

    q3Island *m_island;
    q3ContactConstraintState *m_contacts;
    i32 m_contactCount;
    q3VelocityState *m_velocities;

    bool m_enableFriction;
};

#endif // Q3CONTACTSOLVERCPU_H
