//--------------------------------------------------------------------------------------------------
/**
@file    q3IslandSolver.h

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

#ifndef Q3ISLANDSOLVER_H
#define Q3ISLANDSOLVER_H

class q3Scene;

//--------------------------------------------------------------------------------------------------
// q3IslandSolver
//--------------------------------------------------------------------------------------------------
struct q3IslandSolver
{
    virtual void Solve( q3Scene *scene ) = 0;

    virtual ~q3IslandSolver() {};
};

#endif // Q3ISLANDSOLVER_H
