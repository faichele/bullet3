#ifndef B3_RIGIDBODY_BEHAVIOR_H
#define B3_RIGIDBODY_BEHAVIOR_H

#include "Bullet3Common/shared/b3Float4.h"
#include "Bullet3Common/shared/b3Quat.h"

#define B3_RIGID_BODY_BEHAVIOR_MAX_CONTACTS 10

typedef struct b3RigidBodyPushPullBehavior b3RigidBodyPushPullBehavior_t;

struct b3RigidBodyPushPullBehavior
{
public:
	int m_bodyID;
	int m_ghostObjectID;
	b3Float4 m_bodyPosition;  // Should not be needed, but ConcaveScene demo operates with mesh offsets instead of actual origin transforms.
	b3Quat m_bodyOrientation;
	bool m_perContactPoint;

	b3Float4 m_linearVel;
	b3Float4 m_angularVel;

	b3RigidBodyPushPullBehavior() : m_bodyID(-1), m_ghostObjectID(-1)
	{
		m_bodyPosition = b3MakeFloat4(0, 0, 0, 0);
		m_bodyOrientation = b3Quaternion(0, 0, 0, 1);
		m_linearVel = b3MakeFloat4(0, 0, 0, 0);
		m_angularVel = b3MakeFloat4(0, 0, 0, 0);
		m_perContactPoint = false;
	}

	b3RigidBodyPushPullBehavior(const b3RigidBodyPushPullBehavior& other)
	{
		if (this != &other)
		{
			m_bodyID = other.m_bodyID;
			m_ghostObjectID = other.m_ghostObjectID;
			m_perContactPoint = other.m_perContactPoint;

			m_bodyPosition.x = other.m_bodyPosition.x;
			m_bodyPosition.y = other.m_bodyPosition.y;
			m_bodyPosition.z = other.m_bodyPosition.z;
			m_bodyPosition.w = other.m_bodyPosition.w;

			m_linearVel.x = other.m_linearVel.x;
			m_linearVel.y = other.m_linearVel.y;
			m_linearVel.z = other.m_linearVel.z;
			m_linearVel.w = other.m_linearVel.w;

			m_angularVel.x = other.m_angularVel.x;
			m_angularVel.y = other.m_angularVel.y;
			m_angularVel.z = other.m_angularVel.z;
			m_angularVel.w = other.m_angularVel.w;

			m_bodyOrientation.x = other.m_bodyOrientation.x;
			m_bodyOrientation.y = other.m_bodyOrientation.y;
			m_bodyOrientation.z = other.m_bodyOrientation.z;
			m_bodyOrientation.w = other.m_bodyOrientation.w;
		}
	}

	b3RigidBodyPushPullBehavior& operator=(const b3RigidBodyPushPullBehavior& other)
	{
		if (this != &other)
		{
			m_bodyID = other.m_bodyID;
			m_ghostObjectID = other.m_ghostObjectID;
			m_perContactPoint = other.m_perContactPoint;

			m_bodyPosition.x = other.m_bodyPosition.x;
			m_bodyPosition.y = other.m_bodyPosition.y;
			m_bodyPosition.z = other.m_bodyPosition.z;
			m_bodyPosition.w = other.m_bodyPosition.w;

			m_linearVel.x = other.m_linearVel.x;
			m_linearVel.y = other.m_linearVel.y;
			m_linearVel.z = other.m_linearVel.z;
			m_linearVel.w = other.m_linearVel.w;

			m_angularVel.x = other.m_angularVel.x;
			m_angularVel.y = other.m_angularVel.y;
			m_angularVel.z = other.m_angularVel.z;
			m_angularVel.w = other.m_angularVel.w;

			m_bodyOrientation.x = other.m_bodyOrientation.x;
			m_bodyOrientation.y = other.m_bodyOrientation.y;
			m_bodyOrientation.z = other.m_bodyOrientation.z;
			m_bodyOrientation.w = other.m_bodyOrientation.w;
		}
		return *this;
	}
};

typedef struct b3RigidBodyBehaviorVelocities b3RigidBodyBehaviorVelocities_t;

struct b3RigidBodyBehaviorVelocities
{
	b3Float4 m_linearVel;
	b3Float4 m_angularVel;

	b3Float4 m_linearVelPerContact[B3_RIGID_BODY_BEHAVIOR_MAX_CONTACTS];
	b3Float4 m_angularVelPerContact[B3_RIGID_BODY_BEHAVIOR_MAX_CONTACTS];
};

#endif