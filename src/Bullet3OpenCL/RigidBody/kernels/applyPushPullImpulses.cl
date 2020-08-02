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
							ppLinVel.x += pushPullBehaviors[k].m_linearVel.x;
							ppLinVel.y += pushPullBehaviors[k].m_linearVel.y;
							ppLinVel.z += pushPullBehaviors[k].m_linearVel.z;

							ppAngVel.x += pushPullBehaviors[k].m_angularVel.x;
							ppAngVel.y += pushPullBehaviors[k].m_angularVel.y;
							ppAngVel.z += pushPullBehaviors[k].m_angularVel.z;
						}
					}
				}
			}

			float BT_GPU_ANGULAR_MOTION_THRESHOLD = (0.25f * 3.14159254f);

			//angular velocity
			{
				b3Float4 axis;
				//add some hardcoded angular damping
				bodies[nodeID].m_angVel.x *= angularDamping;
				bodies[nodeID].m_angVel.y *= angularDamping;
				bodies[nodeID].m_angVel.z *= angularDamping;

				b3Float4 angvel = bodies[nodeID].m_angVel;

				angvel.x += ppAngVel.x;
				angvel.y += ppAngVel.y;
				angvel.z += ppAngVel.z;
				
				float fAngle = b3Sqrt(b3Dot3F4(angvel, angvel));

				//limit the angular motion
				if (fAngle * timeStep > BT_GPU_ANGULAR_MOTION_THRESHOLD)
				{
					fAngle = BT_GPU_ANGULAR_MOTION_THRESHOLD / timeStep;
				}
				if (fAngle < 0.001f)
				{
					// use Taylor's expansions of sync function
					axis = angvel * (0.5f * timeStep - (timeStep * timeStep * timeStep) * 0.020833333333f * fAngle * fAngle);
				}
				else
				{
					// sync(fAngle) = sin(c*fAngle)/t
					axis = angvel * (b3Sin(0.5f * fAngle * timeStep) / fAngle);
				}

				b3Quat dorn;
				dorn.x = axis.x;
				dorn.y = axis.y;
				dorn.z = axis.z;
				dorn.w = b3Cos(fAngle * timeStep * 0.5f);
				b3Quat orn0 = bodies[nodeID].m_quat;
				b3Quat predictedOrn = b3QuatMul(dorn, orn0);
				predictedOrn = b3QuatNormalized(predictedOrn);
				bodies[nodeID].m_quat = predictedOrn;
			}

			// Record the push-pull related velocity terms for debugging
			pushPullVelocities[nodeID].m_linearVel = ppLinVel;
			pushPullVelocities[nodeID].m_angularVel = ppAngVel;

			// Correct order is acceleration -> velocity -> position...
			//apply gravity
			bodies[nodeID].m_linVel += gravityAcceleration * timeStep;

			// Apply push-pull behavior linear velocity if applicable: Only add once and subtract again once contact with push-pull enabled object goes away.
			bodies[nodeID].m_linVel.x += ppLinVel.x;
			bodies[nodeID].m_linVel.y += ppLinVel.y;
			bodies[nodeID].m_linVel.z += ppLinVel.z;

			//linear velocity
			bodies[nodeID].m_pos += bodies[nodeID].m_linVel * timeStep;
		}
	}
}