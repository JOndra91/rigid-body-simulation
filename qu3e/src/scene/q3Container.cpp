//--------------------------------------------------------------------------------------------------
/**
@file    q3Container.cpp

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

#include "q3Container.h"

using namespace std;

q3Container::q3Container(q3Scene *scene)
    : m_scene(scene)
    // , m_bodies <q3Body>
    // , m_boxes <vector<q3Box>*>
    // , m_contactList <q3ContactEdge>
    // , m_bodyRefPointers <q3BodyRef*>
    // , m_bodyRef <q3BodyRef>
    {};

q3BodyRef* q3Container::create( const q3BodyDef& def ) {
    q3Body *body;
    q3BodyRef *ref;
    u32 index = m_bodies.size();

    m_bodies.push_back(q3Body(def));
    body = &m_bodies.back();

    m_bodyRefs.push_back(q3BodyRef(m_scene, this));

    ref = &m_bodyRefs.back();

    ref->setContainerIndex(index);
    ref->m_userData = def.userData;

    return ref;
}

void q3Container::remove( q3BodyRef &body ) {
    u32 index = body.m_bodyIndex;
    q3BodyRef *ref;

    for(auto box : body.m_boxes) {
        remove(body, box);
    }

    m_bodies[index] = m_bodies.back();
    ref = m_bodyPtrs[index] = m_bodyPtrs.back();

    assert(ref->body()->m_containerIndex == m_bodies.back().m_containerIndex);

    ref->setContainerIndex(index);

    m_bodies.pop_back();
    m_bodyPtrs.pop_back();
    m_bodyRefs.pop_back();
}

void q3Container::remove( q3BodyRef &body, q3BoxRef &box ) {
    u32 index = box.m_boxIndex;
    q3BoxRef *ref;

    m_boxes[index] = m_boxes.back();
    ref = m_boxPtrs[index] = m_boxPtrs.back();

    ref->setContainerIndex(index);

    body.m_boxes.remove(box);
    m_boxes.pop_back();
    m_boxPtrs.pop_back();
}

void q3Container::clear() {
    m_bodies.clear();
    m_boxes.clear();
    m_bodyRefs.clear();
    m_bodyPtrs.clear();
    m_boxPtrs.clear();
}
