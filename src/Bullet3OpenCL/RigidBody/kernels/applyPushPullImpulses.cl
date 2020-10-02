#pragma OPENCL EXTENSION cl_khr_byte_addressable_store : enable
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyData.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyBehavior.h"

__kernel void findPushPullContactsKernel(__global b3RigidBodyData_t* bodies, 
										 const int numBodies, 
										 float timeStep, 
										 __global int4* collidingPairs, 
										 __global b3RigidBodyPushPullBehavior_t* pushPullBehaviors, 
	                                     __global b3RigidBodyBehaviorVelocities_t* pushPullVelocities, 
	                                     int numCollidingPairs, 
	                                     int numPushPullBehaviors)
{
	int bodyID = get_global_id(0);
	if (bodyID < numBodies)
	{
		// Check which push-pull behaviors to apply for the current bodyID
		b3Float4 ppLinVel = b3MakeFloat4(0, 0, 0, 0);
		b3Float4 ppAngVel = b3MakeFloat4(0, 0, 0, 0);
		if (bodies[bodyID].m_invMass != 0.f)
		{
			for (int k = 0; k < numCollidingPairs; ++k)
			{
				/*ppLinVel.y = collidingPairs[k].x;
				ppAngVel.z = collidingPairs[k].y;*/

				if (collidingPairs[k].x == bodyID || collidingPairs[k].y == bodyID)
				{
					for (int l = 0; l < numPushPullBehaviors; ++l)
					{
						if (pushPullBehaviors[l].m_bodyID == collidingPairs[k].x || pushPullBehaviors[l].m_bodyID == collidingPairs[k].y)
						{
							ppLinVel += pushPullBehaviors[l].m_linearVel;	 
							ppAngVel += pushPullBehaviors[l].m_angularVel;
							/*if (k % 2 == 0)
							{
								ppLinVel.z += 0.5;
								ppAngVel.y -= 0.5;
							}*/
						}
					}
				}
			}

			pushPullVelocities[bodyID].m_linearVel = ppLinVel;
			pushPullVelocities[bodyID].m_angularVel = ppAngVel;
		}
	}
}
