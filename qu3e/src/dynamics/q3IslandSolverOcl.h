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

#include "q3IslandSolver.h"

//--------------------------------------------------------------------------------------------------
// q3IslandSolverOcl
//--------------------------------------------------------------------------------------------------
struct q3IslandSolverOcl : q3IslandSolver
{
    void Solve( q3Scene *scene ) override;
};

#endif // Q3ISLANDSOLVEROCL_H
