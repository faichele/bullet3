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
			// Check if the contact pair concerns the current push-pull behavior
			if (colliding_pairs[k].x == bodyID || colliding_pairs[k].y == bodyID)
			{
				int ppRecipient = -1;
				if (colliding_pairs[k].x == bodyID)
					ppRecipient = colliding_pairs[k].y;
				else if (colliding_pairs[k].y == bodyID)
					ppRecipient = colliding_pairs[k].x;

				if (ppRecipient == -1)
					continue;

				// Apply to all bodies that have contact points with the given bodyID as specified by the b3RigidBodyPushPullBehavior_t entry for the bodyID
				// An additional rotational impulse
				// Either to other bodies' centers of mass
				if (!bodies_behaviors[pushPullBehaviorID].m_perContactPoint)
				{
					// Taken from integrateKernel/b3IntegrateTransforms.h
					if (bodies[ppRecipient].m_invMass != 0.f)
					{
						float BT_GPU_ANGULAR_MOTION_THRESHOLD = (0.25f * 3.14159254f);

						//angular velocity
						{
							b3Float4 axis;
							//add some hardcoded angular damping
							bodies[nodeID].m_angVel.x *= angularDamping;
							bodies[nodeID].m_angVel.y *= angularDamping;
							bodies[nodeID].m_angVel.z *= angularDamping;

							b3Float4 angvel = bodies[nodeID].m_angVel;

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

						// Correct order is acceleratio -> velocity -> position...
						//apply gravity
						bodies[nodeID].m_linVel += gravityAcceleration * timeStep;

						//linear velocity
						bodies[nodeID].m_pos += bodies[nodeID].m_linVel * timeStep;
					}
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
}