#ifndef B3_CPU_PUSH_PULL_BEHAVIOUR_APPLICATION
#define B3_CPU_PUSH_PULL_BEHAVIOUR_APPLICATION

#include "Bullet3Common/b3AlignedObjectArray.h"
#include "Bullet3Collision/BroadPhaseCollision/b3OverlappingPair.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyData.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyBehavior.h"

#include <vector>
#include <map>

class b3PushPullBehaviourApplication
{
	public:
		b3PushPullBehaviourApplication();

		void registerPushPullBehavior(const b3RigidBodyPushPullBehavior&);
		void unregisterPushPullBehavior(const b3RigidBodyPushPullBehavior&);

		void findPushPullContacts(struct b3RigidBodyData* rigidBodies,
								  unsigned int numBodies, float timeStep, 
								  const b3AlignedObjectArray<b3BroadphasePair>& overlappingPairsCPU,
								  b3AlignedObjectArray<b3RigidBodyBehaviorVelocities>& pushPullVelocities);

		void applyPushPullBehavioursCPU(struct b3RigidBodyData* rigidBodies,
										unsigned int numBodies, float timeStep, 
										const b3AlignedObjectArray<b3BroadphasePair>& overlappingPairsCPU,
										const b3AlignedObjectArray<b3RigidBodyPushPullBehavior>& pushPullBehaviors,
										b3AlignedObjectArray<b3RigidBodyBehaviorVelocities>& pushPullVelocities);

		const std::map<int, std::vector<int>>& getRigidBodyToPushPullMap() const;

	private:
		b3AlignedObjectArray<int> m_registeredPushPullBodies;
		b3AlignedObjectArray<int> m_registeredPushPullBodyGhostsObjects;

		b3AlignedObjectArray<b3RigidBodyPushPullBehavior> m_pushPullBehaviors;
		b3AlignedObjectArray<b3RigidBodyBehaviorVelocities> m_hostPushPullVelocities;

		std::map<int, std::vector<int>> m_rbToPPBehaviorMap;
};

#endif