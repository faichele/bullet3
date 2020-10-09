#include "b3CpuPushPullBehaviourApplication.h"

#include <iostream>
#include <vector>
#include <map>

b3PushPullBehaviourApplication::b3PushPullBehaviourApplication()
{

}

const std::map<int, std::vector<int>>& b3PushPullBehaviourApplication::getRigidBodyToPushPullMap() const
{
	return m_rbToPPBehaviorMap;
}

void b3PushPullBehaviourApplication::registerPushPullBehavior(const b3RigidBodyPushPullBehavior& ppBehavior)
{
	int bodyIDIdx = m_registeredPushPullBodies.findBinarySearch(ppBehavior.m_bodyID);
	if (bodyIDIdx == m_registeredPushPullBodies.size())
		m_registeredPushPullBodies.push_back(ppBehavior.m_bodyID);

	int ghostIDIdx = m_registeredPushPullBodyGhostsObjects.findBinarySearch(ppBehavior.m_ghostObjectID);
	if (ghostIDIdx == m_registeredPushPullBodyGhostsObjects.size())
		m_registeredPushPullBodyGhostsObjects.push_back(ppBehavior.m_ghostObjectID);

	m_pushPullBehaviors.push_back(ppBehavior);
}

void b3PushPullBehaviourApplication::unregisterPushPullBehavior(const b3RigidBodyPushPullBehavior& ppBehavior)
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

void b3PushPullBehaviourApplication::findPushPullContacts(struct b3RigidBodyData* rigidBodies,
						  unsigned int numBodies, float timeStep,
						  const b3AlignedObjectArray<b3BroadphasePair>& overlappingPairsCPU,
						  b3AlignedObjectArray<b3RigidBodyBehaviorVelocities>& pushPullVelocities)
{
	m_rbToPPBehaviorMap.clear();
	pushPullVelocities.resize(numBodies);
	for (int l = 0; l < overlappingPairsCPU.size(); ++l)
	{
		for (int k = 0; k < m_pushPullBehaviors.size(); ++k)
		{
			if (m_pushPullBehaviors[k].m_bodyID == overlappingPairsCPU[l].x)
			{
				DEBUG_OUTPUT(std::cout << "Body ID " << overlappingPairsCPU[l].y << " (y) in contact with push-pull rigid: " << m_pushPullBehaviors[k].m_bodyID << std::endl);
				m_rbToPPBehaviorMap[overlappingPairsCPU[l].y].push_back(k);
			}
			else if (m_pushPullBehaviors[k].m_bodyID == overlappingPairsCPU[l].y)
			{
				DEBUG_OUTPUT(std::cout << "Body ID " << overlappingPairsCPU[l].x << " (x) in contact with push-pull rigid: " << m_pushPullBehaviors[k].m_bodyID << std::endl);
				m_rbToPPBehaviorMap[overlappingPairsCPU[l].x].push_back(k);
			}
		}
	}

	for (auto it = m_rbToPPBehaviorMap.begin(); it != m_rbToPPBehaviorMap.end(); ++it)
	{
		b3Vector3 ppLinAcc = b3MakeVector3(0, 0, 0);
		b3Vector3 ppAngAcc = b3MakeVector3(0, 0, 0);

		b3Vector3 ppLinVel = b3MakeVector3(0, 0, 0);
		b3Vector3 ppAngVel = b3MakeVector3(0, 0, 0);

		int velsAdded = 0;
		for (size_t k = 0; k < it->second.size(); ++k)
		{
			// if (!m_pushPullBehaviors[m_rbToPPBehaviorMap[it->first][k]].m_perContactPoint)
			{
				DEBUG_OUTPUT(std::cout << "Adding acceleration for body " << it->first << ": linear = (" << m_pushPullBehaviors[m_rbToPPBehaviorMap[it->first][k]].m_linearAcc.x << "," << m_pushPullBehaviors[m_rbToPPBehaviorMap[it->first][k]].m_linearAcc.y << "," << m_pushPullBehaviors[m_rbToPPBehaviorMap[it->first][k]].m_linearAcc.z << ");"
						  << " angular = " << m_pushPullBehaviors[m_rbToPPBehaviorMap[it->first][k]].m_angularAcc.x << "," << m_pushPullBehaviors[m_rbToPPBehaviorMap[it->first][k]].m_angularAcc.y << "," << m_pushPullBehaviors[m_rbToPPBehaviorMap[it->first][k]].m_angularAcc.z << ")" << std::endl);

				ppLinAcc.x += m_pushPullBehaviors[m_rbToPPBehaviorMap[it->first][k]].m_linearAcc.x;
				ppLinAcc.y += m_pushPullBehaviors[m_rbToPPBehaviorMap[it->first][k]].m_linearAcc.y;
				ppLinAcc.z += m_pushPullBehaviors[m_rbToPPBehaviorMap[it->first][k]].m_linearAcc.z;

				ppAngAcc.x += m_pushPullBehaviors[m_rbToPPBehaviorMap[it->first][k]].m_angularAcc.x;
				ppAngAcc.y += m_pushPullBehaviors[m_rbToPPBehaviorMap[it->first][k]].m_angularAcc.y;
				ppAngAcc.z += m_pushPullBehaviors[m_rbToPPBehaviorMap[it->first][k]].m_angularAcc.z;
				
				ppLinVel.x += m_pushPullBehaviors[m_rbToPPBehaviorMap[it->first][k]].m_linearVel.x;
				ppLinVel.y += m_pushPullBehaviors[m_rbToPPBehaviorMap[it->first][k]].m_linearVel.y;
				ppLinVel.z += m_pushPullBehaviors[m_rbToPPBehaviorMap[it->first][k]].m_linearVel.z;

				ppAngVel.x += m_pushPullBehaviors[m_rbToPPBehaviorMap[it->first][k]].m_angularVel.x;
				ppAngVel.y += m_pushPullBehaviors[m_rbToPPBehaviorMap[it->first][k]].m_angularVel.y;
				ppAngVel.z += m_pushPullBehaviors[m_rbToPPBehaviorMap[it->first][k]].m_angularVel.z;
				
				velsAdded++;
			}
		}

		std::cout << "Setting body pushPullVelocities[" << it->first << "] with " << velsAdded << " ppVels; m_angularVel = (" << ppAngVel.x << "," << ppAngVel.y << "," << ppAngVel.z << "); "
				  << "m_linearVel = (" << ppLinVel.x << ", " << ppLinVel.y << ", " << ppLinVel.z << ")"
				  << std::endl;

		pushPullVelocities[it->first].m_angularAcc = ppAngAcc;
		pushPullVelocities[it->first].m_linearAcc = ppLinAcc;

		pushPullVelocities[it->first].m_angularVel = ppLinVel;
		pushPullVelocities[it->first].m_linearVel = ppLinVel;
	}

	/*auto it = rbToPPBehaviorMap.begin();
	auto end = rbToPPBehaviorMap.end();

	while (it != end)
	{
		auto key = it->first;
		b3Vector3 ppLinAcc = b3MakeVector3(0, 0, 0);
		b3Vector3 ppAngAcc = b3MakeVector3(0, 0, 0);

		int velsAdded = 0;
		do
		{
			if (it++ == end)
				break;

			DEBUG_OUTPUT(std::cout << "Adding acceleration for body " << it->second << ": linear = (" << m_pushPullBehaviors[it->second].m_linearAcc.x << "," << m_pushPullBehaviors[it->second].m_linearAcc.y << "," << m_pushPullBehaviors[it->second].m_linearAcc.z << ");"
					  << " angular = " << m_pushPullBehaviors[it->second].m_angularAcc.x << "," << m_pushPullBehaviors[it->second].m_angularAcc.y << "," << m_pushPullBehaviors[it->second].m_angularAcc.z << ")" << std::endl);

			ppLinAcc.x += m_pushPullBehaviors[it->second].m_linearAcc.x;
			ppLinAcc.y += m_pushPullBehaviors[it->second].m_linearAcc.y;
			ppLinAcc.z += m_pushPullBehaviors[it->second].m_linearAcc.z;

			ppAngAcc.x += m_pushPullBehaviors[it->second].m_angularAcc.x;
			ppAngAcc.y += m_pushPullBehaviors[it->second].m_angularAcc.y;
			ppAngAcc.z += m_pushPullBehaviors[it->second].m_angularAcc.z;
			velsAdded++;
		} while (it->first == key);

		DEBUG_OUTPUT(std::cout << "Setting body pushPullVelocities[" << key << "] with " << velsAdded << " ppAccels; m_angularAcc = (" << ppAngAcc.x << "," << ppAngAcc.y << "," << ppAngAcc.z << "); "
				  << "m_linearAcc = (" << ppLinAcc.x << ", " << ppLinAcc.y << ", " << ppLinAcc.z << ")"
				  << std::endl);

		pushPullVelocities[key].m_angularAcc = ppAngAcc;
		pushPullVelocities[key].m_linearAcc = ppLinAcc;

		pushPullVelocities[key].m_angularVel = ppAngAcc * timeStep;
		pushPullVelocities[key].m_linearVel = ppLinAcc * timeStep;
	}*/
}

void b3PushPullBehaviourApplication::applyPushPullBehavioursCPU(struct b3RigidBodyData* rigidBodies,
																unsigned int numBodies, float timeStep,
																const b3AlignedObjectArray<b3BroadphasePair>& overlappingPairsCPU,
																const b3AlignedObjectArray<b3RigidBodyPushPullBehavior>& pushPullBehaviors,
																b3AlignedObjectArray<b3RigidBodyBehaviorVelocities>& pushPullVelocities)
{
	std::cout << "Applying PushPullBehaviours: " << m_pushPullBehaviors.size() << "; timeStep = " << timeStep << std::endl;

	for (int k = 0; k < numBodies; ++k)
	{
		if (rigidBodies[k].m_invMass != 0.f)
		{
			std::cout << "Appling push-pull velocities to body " << k
					  << ": linear  = (" << pushPullVelocities[k].m_linearVel.x << "," << pushPullVelocities[k].m_linearVel.y << "," << pushPullVelocities[k].m_linearVel.z << ")"
					  << ": angular = (" << pushPullVelocities[k].m_angularVel.x << "," << pushPullVelocities[k].m_angularVel.y << "," << pushPullVelocities[k].m_angularVel.z << ")" << std::endl;

			// apply velocity from push-pull behaviors
			rigidBodies[k].m_linVel += pushPullVelocities[k].m_linearVel;
			rigidBodies[k].m_angVel += pushPullVelocities[k].m_angularVel;
		}
	}
}
