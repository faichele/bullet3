#ifndef B3_CPU_PUSH_PULL_BEHAVIOUR_APPLICATION
#define B3_CPU_PUSH_PULL_BEHAVIOUR_APPLICATION

#include "Bullet3Common/b3AlignedObjectArray.h"
#include "Bullet3Collision/BroadPhaseCollision/b3OverlappingPair.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyData.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyBehavior.h"

class b3CpuPushPullBehaviourApplication
{
	public:
		b3CpuPushPullBehaviourApplication();

		void registerPushPullBehavior(const b3RigidBodyPushPullBehavior&);
		void unregisterPushPullBehavior(const b3RigidBodyPushPullBehavior&);

		void applyPushPullBehaviours(struct b3RigidBodyData* rigidBodies,
									 unsigned int numBodies, float timeStep, float angularDamp,
									 const b3Vector3& gravity,
									 const b3AlignedObjectArray<b3BroadphasePair>& overlappingPairsCPU,
									 const b3AlignedObjectArray<b3RigidBodyPushPullBehavior>& pushPullBehaviors,
									 b3AlignedObjectArray<b3RigidBodyBehaviorVelocities>& pushPullVelocities);

	private:
		b3AlignedObjectArray<int> m_registeredPushPullBodies;
		b3AlignedObjectArray<int> m_registeredPushPullBodyGhostsObjects;

		b3AlignedObjectArray<b3RigidBodyPushPullBehavior> m_pushPullBehaviors;
		b3AlignedObjectArray<b3RigidBodyBehaviorVelocities> m_hostPushPullVelocities;

};

#endif