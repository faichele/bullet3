#include "b3CpuPushPullBehaviourApplication.h"

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

void b3CpuPushPullBehaviourApplication::applyPushPullBehaviours()
{
	for (int k = 0; k < m_pushPullBehaviors.size(); ++k)
	{

	}
}
