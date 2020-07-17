#ifndef B3_RIGIDBODY_BEHAVIOR_H
#define B3_RIGIDBODY_BEHAVIOR_H

#include "Bullet3Common/shared/b3Float4.h"

#define B3_RIGID_BODY_BEHAVIOR_MAX_CONTACTS 10

typedef struct b3RigidBodyPushPullBehavior b3RigidBodyPushPullBehavior_t;

struct b3RigidBodyPushPullBehavior
{
	int m_bodyID;
	b3Float4 m_bodyPosition; // Should not be needed, but ConcaveScene demo operates with mesh offsets instead of actual origin transforms.
	b3Quaternion m_bodyOrientation;
	bool m_perContactPoint;

	b3Float4 m_linearVel;
	b3Float4 m_angularVel;
};

typedef struct b3RigidBodyBehaviorVelocities b3RigidBodyBehaviorVelocities_t;

struct b3RigidBodyBehaviorVelocities
{
	int m_bodyID;
	b3Float4 m_linearVel;
	b3Float4 m_angularVel;

	b3Float4 m_linearVelPerContact[B3_RIGID_BODY_BEHAVIOR_MAX_CONTACTS];
	b3Float4 m_angularVelPerContact[B3_RIGID_BODY_BEHAVIOR_MAX_CONTACTS];
};

#endif