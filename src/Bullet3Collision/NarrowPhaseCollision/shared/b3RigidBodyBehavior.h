#ifndef B3_RIGIDBODY_BEHAVIOR_H
#define B3_RIGIDBODY_BEHAVIOR_H

#include "Bullet3Common/shared/b3Float4.h"
#include "Bullet3Common/shared/b3Quat.h"

#define B3_RIGID_BODY_BEHAVIOR_MAX_CONTACTS 10

typedef struct b3RigidBodyPushPullBehavior b3RigidBodyPushPullBehavior_t;

struct b3RigidBodyPushPullBehavior
{
	int m_bodyID;
	int m_ghostObjectID;
	b3Float4 m_bodyPosition;  // Should not be needed, but ConcaveScene demo operates with mesh offsets instead of actual origin transforms.
	b3Quat m_bodyOrientation;
	bool m_perContactPoint;

	b3Float4 m_linearVel;
	b3Float4 m_angularVel;

	b3Float4 m_linearAcc;
	b3Float4 m_angularAcc;
};

typedef struct b3RigidBodyBehaviorVelocities b3RigidBodyBehaviorVelocities_t;

struct b3RigidBodyBehaviorVelocities
{
	b3Float4 m_linearVel;
	b3Float4 m_angularVel;

	b3Float4 m_linearAcc;
	b3Float4 m_angularAcc;

	b3Float4 m_linearVelPerContact[B3_RIGID_BODY_BEHAVIOR_MAX_CONTACTS];
	b3Float4 m_angularVelPerContact[B3_RIGID_BODY_BEHAVIOR_MAX_CONTACTS];

	b3Float4 m_linearVelPerContactFriction[B3_RIGID_BODY_BEHAVIOR_MAX_CONTACTS];
	b3Float4 m_angularVelPerContactFriction[B3_RIGID_BODY_BEHAVIOR_MAX_CONTACTS];
};

#endif