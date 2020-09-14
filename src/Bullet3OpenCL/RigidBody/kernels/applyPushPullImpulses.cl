#pragma OPENCL EXTENSION cl_khr_byte_addressable_store : enable
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyData.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyBehavior.h"

__kernel void applyPushPullImpulsesKernel(__global b3RigidBodyData_t* bodies, const int numNodes, float timeStep, float angularDamping, float4 gravityAcceleration, __global int4* collidingPairs, __global b3RigidBodyPushPullBehavior_t* pushPullBehaviors, __global b3RigidBodyBehaviorVelocities_t* pushPullVelocities, int numCollidingPairs, int numPushPullBehaviors)
{
	// Taken directly from b3IntegrateTransforms.h (integrateSingleTransform function)
	// This adds the velocities (linear and angular) for push-pull behaviors in addition to the usual integration steps per body

	int nodeID = get_global_id(0);

	if (nodeID < numNodes)
	{
		if (bodies[nodeID].m_invMass != 0.f)
		{
			// Check if there is a push-pull behavior to apply for the current body
			b3Float4 ppLinVel = b3MakeFloat4(0, 0, 0, 0);
			b3Float4 ppAngVel = b3MakeFloat4(0, 0, 0, 0);
			for (int i = 0; i < numCollidingPairs; ++i)
			{
				// First step: See which contact pairs the given nodeID matches
				if (collidingPairs[i].x == nodeID || collidingPairs[i].y == nodeID)
				{
					// Second step: See which push-pull behaviors have bodyIDs that are involved in a contact with the given nodeID
					// Sum up linear and angular velocities for all push-pull enabled bodies to be applied in one go
					for (int k = 0; k < numPushPullBehaviors; ++k)
					{
						if (pushPullBehaviors[k].m_bodyID == collidingPairs[i].x || pushPullBehaviors[k].m_bodyID == collidingPairs[i].y)
						{
							ppLinVel.x += pushPullBehaviors[k].m_linearAcc.x * timeStep;
							ppLinVel.y += pushPullBehaviors[k].m_linearAcc.y * timeStep;
							ppLinVel.z += pushPullBehaviors[k].m_linearAcc.z * timeStep;

							ppAngVel.x += pushPullBehaviors[k].m_angularAcc.x * timeStep;
							ppAngVel.y += pushPullBehaviors[k].m_angularAcc.y * timeStep;
							ppAngVel.z += pushPullBehaviors[k].m_angularAcc.z;
						}
					}
				}
			}

			// Apply push-pull behavior angular velocity
			bodies[nodeID].m_angVel += ppAngVel;
			
			// Apply push-pull behavior linear velocity
			bodies[nodeID].m_linVel.x += ppLinVel.x;
			bodies[nodeID].m_linVel.y += ppLinVel.y;
			bodies[nodeID].m_linVel.z += ppLinVel.z;

			// Record the push-pull related velocity terms for debugging
			pushPullVelocities[nodeID].m_linearVel = ppLinVel;
			pushPullVelocities[nodeID].m_angularVel = ppAngVel;
		}
	}
}