//--------------------------------------------------------------------------------------------------
/**
@file    q3Container.h

@author    Ondřej Janošík
@date    11/5/2016

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

#ifndef Q3BODY_CONTAINER_H
#define Q3BODY_CONTAINER_H

#include <vector>
#include <list>
#include "../math/q3Math.h"
#include "../math/q3Transform.h"
#include "../dynamics/q3Body.h"
#include "../collision/q3Box.h"

//--------------------------------------------------------------------------------------------------
// q3Container
//--------------------------------------------------------------------------------------------------
class q3Scene;
struct q3BodyDef;
class q3BoxDef;
struct q3ContactEdge;
class q3Render;

using std::vector;
using std::list;
class q3Container {
    q3Scene *m_scene;

    vector<q3Body> m_bodies;
    vector<q3Box> m_boxes;
    vector<q3BodyRef*> m_bodyPtrs;
    vector<q3BoxRef*> m_boxPtrs;

    friend class q3BodyRef;
    friend class q3BoxRef;
    friend class q3Scene;
    friend class q3ContactManagerOcl;
    friend struct q3IslandSolverOcl;

public:
    q3Container(q3Scene *scene);

    q3BodyRef* create( const q3BodyDef& def );
    void remove( q3BodyRef *body );
    void remove( q3BodyRef *body, q3BoxRef *box );
    void clear();

    inline const vector<q3BodyRef*>& bodies() const {
        return m_bodyPtrs;
    }

    inline q3BodyRef* getBodyRef(u32 index) const {
        return m_bodyPtrs[index];
    }

    inline q3BoxRef* getBoxRef(u32 index) const {
        return m_boxPtrs[index];
    }

    inline q3BodyRef* getRef(const q3Body* body) const {
        return getBodyRef(body->m_bodyIndex);
    }

    inline q3BoxRef* getRef(const q3Box* box) const {
        return getBoxRef(box->m_boxIndex);
    }
};


#endif // Q3BODY_CONTAINER_H
