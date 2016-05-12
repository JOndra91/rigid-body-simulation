//--------------------------------------------------------------------------------------------------
/**
@file    q3Box.h

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

#ifndef Q3BOX_H
#define Q3BOX_H

#include "../math/q3Vec3.h"
#include "../math/q3Mat3.h"
#include "../math/q3Transform.h"
#include "../debug/q3Render.h"

class q3Body;
class q3BodyRef;
class q3Container;

//--------------------------------------------------------------------------------------------------
// q3MassData
//--------------------------------------------------------------------------------------------------
struct q3MassData
{
    q3Mat3 inertia;
    q3Vec3 center;
    r32 mass;
};

//--------------------------------------------------------------------------------------------------
// q3Box
//--------------------------------------------------------------------------------------------------
struct q3Box
{
    q3Transform local;
    q3Vec3 e;

    // q3Box* next;
    // class q3Body* body;
    r32 friction;
    r32 restitution;
    r32 density;
    i32 broadPhaseIndex;
    // mutable void* userData;
    mutable bool sensor;
    u32 m_containerIndex;
    u32 m_bodyIndex;

    void SetSensor( bool isSensor );

    bool TestPoint( const q3Transform& tx, const q3Vec3& p ) const;
    bool Raycast( const q3Transform& tx, q3RaycastData* raycast ) const;
    void ComputeAABB( const q3Transform& tx, q3AABB* aabb ) const;
    void ComputeMass( q3MassData* md ) const;
    void Render( const q3Transform& tx, bool awake, q3Render* render ) const;
};

//--------------------------------------------------------------------------------------------------
// q3BoxRef
//--------------------------------------------------------------------------------------------------
class q3BoxRef
{
    q3Container *m_container;
    // q3Scene *m_scene;
    mutable void *m_userData;
    u32 m_bodyIndex;
    u32 m_boxIndex;

    q3BoxRef(q3Container *container);

    void setBodyIndex(u32 bodyIndex);
    void setContainerIndex(u32 index);

    friend class q3Container;
    friend class q3BodyRef;

public:

    q3Box* box() const;
    q3Body* body() const;

    void SetUserdata( void* data ) const;
    void* GetUserdata( ) const;

    inline void SetSensor( bool isSensor ) {
        box()->SetSensor(isSensor);
    }

    q3BodyRef* getBodyRef() const;

    bool operator==( const q3BoxRef& other ) {
        return m_bodyIndex == other.m_bodyIndex
            && m_boxIndex == other.m_boxIndex;
    }

    //-------------------------------------------------------------------------

    inline bool TestPoint( const q3Transform& tx, const q3Vec3& p ) const {
        return box()->TestPoint( tx, p );
    }

    inline bool Raycast( const q3Transform& tx, q3RaycastData* raycast ) const {
        return box()->Raycast( tx, raycast );
    }

    inline void ComputeAABB( const q3Transform& tx, q3AABB* aabb ) const {
        box()->ComputeAABB( tx, aabb );
    }

    inline void ComputeMass( q3MassData* md ) const {
        box()->ComputeMass( md );
    }

    inline void Render( const q3Transform& tx, bool awake, q3Render* render ) const {
        box()->Render( tx, awake, render );
    }

    //-------------------------------------------------------------------------
};

//--------------------------------------------------------------------------------------------------
// q3BoxDef
//--------------------------------------------------------------------------------------------------
class q3BoxDef
{
public:
    q3BoxDef( )
    {
        // Common default values
        m_friction = r32( 0.4 );
        m_restitution = r32( 0.2 );
        m_density = r32( 1.0 );
        m_sensor = false;
    }

    void Set( const q3Transform& tx, const q3Vec3& extents );

    void SetFriction( r32 friction );
    void SetRestitution( r32 restitution );
    void SetDensity( r32 density );
    void SetSensor( bool sensor );

private:
    q3Transform m_tx;
    q3Vec3 m_e;

    r32 m_friction;
    r32 m_restitution;
    r32 m_density;
    bool m_sensor;

    friend class q3Body;
    friend class q3BodyRef;
};

#include "q3Box.inl"


#endif // Q3BOX_H
