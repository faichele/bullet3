#pragma OPENCL EXTENSION cl_khr_byte_addressable_store : enable
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyData.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyBehavior.h"

__kernel void applyPushPullImpulsesKernel(__global int4* colliding_pairs, __global b3RigidBodyData_t* bodies, __global b3RigidBodyPushPullBehavior_t* bodies_behaviors, float timeStep, int numBodies, int numCollidingPairs, int numPushPullBehaviors)
{
	int pushPullBehaviorID = get_global_id(0);
	if (pushPullBehaviorID < numPushPullBehaviors)
	{
		int bodyID = bodies_behaviors[pushPullBehaviorID].m_bodyID;
		for (int k = 0; k < numCollidingPairs; ++k)
		{
			// Apply to all bodies that have contact points with the given bodyID as specified by the b3RigidBodyPushPullBehavior_t entry for the bodyID
			// An additional rotational impulse
			// Either to other bodies' centers of mass
			if (!bodies_behaviors[pushPullBehaviorID].m_perContactPoint)
			{

			}
			// Or per contact point
			// An additional translational impulse
			// Either to other bodies' centers of mass
			else
			{

			}
		}
	}
}