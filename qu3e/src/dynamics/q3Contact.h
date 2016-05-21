//--------------------------------------------------------------------------------------------------
/**
@file    q3Contact.h

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

#ifndef Q3CONTACT_H
#define Q3CONTACT_H

#include "../math/q3Math.h"
#include "../common/q3Settings.h"
#include "../collision/q3Box.h"
#include "../dynamics/q3Body.h"
#include "../collision/q3Collide.h"

//--------------------------------------------------------------------------------------------------
// q3Contact
//--------------------------------------------------------------------------------------------------
class q3Body;
class q3BodyRef;
struct q3Box;
class q3BoxRef;
struct q3ContactConstraint;

// Restitution mixing. The idea is to use the maximum bounciness, so bouncy
// objects will never not bounce during collisions.
inline r32 q3MixRestitution( const q3Box* A, const q3Box* B )
{
    return q3Max( A->restitution, B->restitution );
}

// Friction mixing. The idea is to allow a very low friction value to
// drive down the mixing result. Example: anything slides on ice.
inline r32 q3MixFriction( const q3Box* A, const q3Box* B )
{
    return std::sqrt( A->friction * B->friction );
}

union q3FeaturePair
{
    struct
    {
        u8 inR;
        u8 outR;
        u8 inI;
        u8 outI;
    };

    i32 key;
} PACKED;


struct q3ClipVertex
{
    q3ClipVertex( )
    {
        f.key = ~0;
    }

    q3Vec3 v;
    q3FeaturePair f;
} ALIGNED;

struct q3Contact
{
    q3Vec3 position;            // World coordinate of contact
    r32 penetration;            // Depth of penetration from collision
    r32 normalImpulse;            // Accumulated normal impulse
    r32 tangentImpulse[ 2 ];    // Accumulated friction impulse
    r32 bias;                    // Restitution + baumgarte
    r32 normalMass;                // Normal constraint mass
    r32 tangentMass[ 2 ];        // Tangent constraint mass
    q3FeaturePair fp;            // Features on A and B for this contact
    u8 warmStarted;                // Used for debug rendering

    void print(const char *padding) {
        fprintf(stderr, "%sContact:\n", padding);
        fprintf(stderr, "%s  position: vec3(%.10f, %.10f, %.10f)\n", padding, position.x, position.y, position.z);
        fprintf(stderr, "%s  penetration: %.10f\n", padding, penetration);
        fprintf(stderr, "%s  normalImpulse: %.10f\n", padding, normalImpulse);
        fprintf(stderr, "%s  tangentImpulse[0]: %.10f\n", padding, tangentImpulse[0]);
        fprintf(stderr, "%s  tangentImpulse[1]: %.10f\n", padding, tangentImpulse[1]);
        fprintf(stderr, "%s  bias: %.10f\n", padding, bias);
        fprintf(stderr, "%s  normalMass: %.10f\n", padding, normalMass);
        fprintf(stderr, "%s  tangentMass[0]: %.10f\n", padding, tangentMass[0]);
        fprintf(stderr, "%s  tangentMass[1]: %.10f\n", padding, tangentMass[1]);
        fprintf(stderr, "%s  fp: 0x%x\n", padding, fp.key);
        fprintf(stderr, "%s  warmStarted: %d\n", padding, warmStarted);
    };
} ALIGNED;

struct q3Manifold
{
    void SetPair( q3BoxRef *a, q3BoxRef *b );

    q3BoxRef *A;
    q3BoxRef *B;

    q3Vec3 normal;                // From A to B
    q3Vec3 tangentVectors[ 2 ];    // Tangent vectors
    q3Contact contacts[ 8 ];
    i32 contactCount;

    q3Manifold* next;
    q3Manifold* prev;

    bool sensor;

    void print(const char *padding) {
        fprintf(stderr, "%sManifold:\n", padding);
        fprintf(stderr, "%s  boxA: %d\n", padding, A->getBoxIndex());
        fprintf(stderr, "%s  boxB: %d\n", padding, B->getBoxIndex());
        fprintf(stderr, "%s  normal: vec3(%.10f, %.10f, %.10f)\n", padding, normal.x, normal.y, normal.z);
        fprintf(stderr, "%s  tangentVectors[0]: vec3(%.10f, %.10f, %.10f)\n", padding, tangentVectors[0].x, tangentVectors[0].y, tangentVectors[0].z);
        fprintf(stderr, "%s  tangentVectors[1]: vec3(%.10f, %.10f, %.10f)\n", padding, tangentVectors[1].x, tangentVectors[1].y, tangentVectors[1].z);
        fprintf(stderr, "%s  normal: vec3(%.10f, %.10f, %.10f)\n", padding, normal.x, normal.y, normal.z);
        fprintf(stderr, "%s  sensor: %d\n", padding, sensor);
        fprintf(stderr, "%s  contactCount: %d\n", padding, contactCount);
        for(int i = 0; i < contactCount; ++i) {
            contacts[i].print(padding - 2);
        }
    };
};

struct q3ContactEdge
{
    q3BodyRef *other;
    q3ContactConstraint *constraint;
    q3ContactEdge* next;
    q3ContactEdge* prev;
};

struct q3ContactConstraint
{
    void SolveCollision( void );

    q3BoxRef *A, *B;
    q3BodyRef *bodyA, *bodyB;

    q3ContactEdge edgeA;
    q3ContactEdge edgeB;
    q3ContactConstraint* next;
    q3ContactConstraint* prev;

    r32 friction;
    r32 restitution;

    q3Manifold manifold;

    enum
    {
        eColliding    = 0x00000001, // Set when contact collides during a step
        eWasColliding = 0x00000002, // Set when two objects stop colliding
        eIsland       = 0x00000004, // For internal marking during island forming
        eRemove       = 0x00000008,
    };

    i32 m_flags;

    void print(const char *padding) {
        fprintf(stderr, "%sContactConstraint:\n", padding);
        fprintf(stderr, "%s  boxA: %d\n", padding, A->getBoxIndex());
        fprintf(stderr, "%s  boxB: %d\n", padding, B->getBoxIndex());
        fprintf(stderr, "%s  bodyA: %d\n", padding, A->getBodyIndex());
        fprintf(stderr, "%s  bodyB: %d\n", padding, B->getBodyIndex());
        fprintf(stderr, "%s  friction: %.10f\n", padding, friction);
        fprintf(stderr, "%s  restitution: %.10f\n", padding, restitution);
        fprintf(stderr, "%s  m_flags: 0x%x\n", padding, m_flags);
        manifold.print(padding - 2);
    };

    friend class q3ContactManager;
    friend class q3Scene;
    friend struct q3Island;
    friend struct q3ContactSolver;
};

#endif // Q3CONTACT_H
