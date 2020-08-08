#include "b3CpuPushPullBehaviourApplication.h"

#include <iostream>
#include <vector>
#include <map>

b3CpuPushPullBehaviourApplication::b3CpuPushPullBehaviourApplication()
{
}

void b3CpuPushPullBehaviourApplication::registerPushPullBehavior(const b3RigidBodyPushPullBehavior& ppBehavior)
{
	int bodyIDIdx = m_registeredPushPullBodies.findBinarySearch(ppBehavior.m_bodyID);
	if (bodyIDIdx == m_registeredPushPullBodies.size())
		m_registeredPushPullBodies.push_back(ppBehavior.m_bodyID);

	int ghostIDIdx = m_registeredPushPullBodyGhostsObjects.findBinarySearch(ppBehavior.m_ghostObjectID);
	if (ghostIDIdx == m_registeredPushPullBodyGhostsObjects.size())
		m_registeredPushPullBodyGhostsObjects.push_back(ppBehavior.m_ghostObjectID);

	m_pushPullBehaviors.push_back(ppBehavior);
}

void b3CpuPushPullBehaviourApplication::unregisterPushPullBehavior(const b3RigidBodyPushPullBehavior& ppBehavior)
{
	int bodyIDIdx = m_registeredPushPullBodies.findBinarySearch(ppBehavior.m_bodyID);
	if (bodyIDIdx < m_registeredPushPullBodies.size())
		m_registeredPushPullBodies.removeAtIndex(bodyIDIdx);

	int ghostIDIdx = m_registeredPushPullBodyGhostsObjects.findBinarySearch(ppBehavior.m_ghostObjectID);
	if (ghostIDIdx < m_registeredPushPullBodyGhostsObjects.size())
		m_registeredPushPullBodyGhostsObjects.removeAtIndex(ghostIDIdx);

	// Fine since when registering all array entries are entered in the same positions
	m_pushPullBehaviors.removeAtIndex(bodyIDIdx);
}

void b3CpuPushPullBehaviourApplication::applyPushPullBehaviours(struct b3RigidBodyData* rigidBodies,
																unsigned int numBodies, float timeStep, float angularDamp,
																const b3Vector3& gravity,
																const b3AlignedObjectArray<b3BroadphasePair>& overlappingPairsCPU,
																const b3AlignedObjectArray<b3RigidBodyPushPullBehavior>& pushPullBehaviors,
																b3AlignedObjectArray<b3RigidBodyBehaviorVelocities>& pushPullVelocities)
{
	pushPullVelocities.resize(numBodies);
	std::cout << "Applying PushPullBehaviours: " << m_pushPullBehaviors.size() << std::endl;

	std::multimap<int, int> rbToPPBehaviorMap;

	for (int l = 0; l < overlappingPairsCPU.size(); ++l)
	{
		for (int k = 0; k < m_pushPullBehaviors.size(); ++k)
		{
			if (m_pushPullBehaviors[k].m_bodyID == overlappingPairsCPU[l].x)
			{
				std::cout << "Body ID " << overlappingPairsCPU[l].y << " (y) in contact with push-pull rigid: " << m_pushPullBehaviors[k].m_bodyID << std::endl;
				rbToPPBehaviorMap.insert(std::make_pair(overlappingPairsCPU[l].y, k));
			}
			else if (m_pushPullBehaviors[k].m_bodyID == overlappingPairsCPU[l].y)
			{
				std::cout << "Body ID " << overlappingPairsCPU[l].x << " (x) in contact with push-pull rigid: " << m_pushPullBehaviors[k].m_bodyID << std::endl;
				rbToPPBehaviorMap.insert(std::make_pair(overlappingPairsCPU[l].x, k));
			}
		}
	}

	auto it = rbToPPBehaviorMap.begin();
	auto end = rbToPPBehaviorMap.end();

	while (it != end)
	{
		auto key = it->first;
		b3Vector3 ppLinVel = b3MakeVector3(0, 0, 0);
		b3Vector3 ppAngVel = b3MakeVector3(0, 0, 0);

		do
		{
			if (++it == end)
				break;

			ppLinVel.x += m_pushPullBehaviors[it->second].m_linearAcc.x;
			ppLinVel.y += m_pushPullBehaviors[it->second].m_linearAcc.y;
			ppLinVel.z += m_pushPullBehaviors[it->second].m_linearAcc.z;

			ppAngVel.x += m_pushPullBehaviors[it->second].m_angularAcc.x;
			ppAngVel.y += m_pushPullBehaviors[it->second].m_angularAcc.y;
			ppAngVel.z += m_pushPullBehaviors[it->second].m_angularAcc.z;

		} while (it->first == key);

		pushPullVelocities[key].m_angularVel = ppAngVel;
		pushPullVelocities[key].m_linearVel = ppLinVel;
	}

	for (int k = 0; k < numBodies; ++k)
	{
		if (rigidBodies[k].m_invMass != 0.f)
		{
			float BT_GPU_ANGULAR_MOTION_THRESHOLD = (0.25f * 3.14159254f);

			//angular velocity
			{
				b3Float4 axis;

				//add some hardcoded angular damping
				rigidBodies[k].m_angVel.x *= angularDamp;
				rigidBodies[k].m_angVel.y *= angularDamp;
				rigidBodies[k].m_angVel.z *= angularDamp;

				b3Float4 angvel = rigidBodies[k].m_angVel;

				// Add angular velocity from push-pull behaviors
				angvel += pushPullVelocities[k].m_angularVel;

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
				b3Quat orn0 = rigidBodies[k].m_quat;
				b3Quat predictedOrn = b3QuatMul(dorn, orn0);
				predictedOrn = b3QuatNormalized(predictedOrn);
				rigidBodies[k].m_quat = predictedOrn;
			}

			// Correct order is acceleration -> velocity -> position...
			//apply gravity
			rigidBodies[k].m_linVel += gravity * timeStep;

			// apply velocity from push-pull behaviors

			rigidBodies[k].m_linVel += pushPullVelocities[k].m_linearVel * timeStep;

			//linear velocity
			rigidBodies[k].m_pos += rigidBodies[k].m_linVel * timeStep;
		}
	}
}
