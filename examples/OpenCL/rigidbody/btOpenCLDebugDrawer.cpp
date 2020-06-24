#include "btOpenCLDebugDrawer.h"

#include <CommonInterfaces/CommonRenderInterface.h>

void OpenCLDebugDrawer::drawDebugDrawerLines()
{
	if (m_hashedLines.size())
	{
		for (int i = 0; i < m_hashedLines.size(); i++)
		{
			ColorWidth cw = m_hashedLines.getKeyAtIndex(i);
			int index = *m_hashedLines.getAtIndex(i);
			int stride = sizeof(btVector3FloatData);
			const float* positions = &m_sortedLines[index][0].m_floats[0];
			int numPoints = m_sortedLines[index].size();
			const unsigned int* indices = &m_sortedIndices[index][0];
			int numIndices = m_sortedIndices[index].size();
			m_guiHelper->getRenderInterface()->drawLines(positions, cw.m_color.m_floats, numPoints, stride, indices, numIndices, cw.width);
		}
	}
}

OpenCLDebugDrawer::OpenCLDebugDrawer(GUIHelperInterface* guiHelper)
	: m_guiHelper(guiHelper),
	  m_debugMode(0)
{

}

OpenCLDebugDrawer::~OpenCLDebugDrawer()
{

}

void OpenCLDebugDrawer::drawLine(const btVector3& from, const btVector3& to, const btVector3& color)
{
	{
		ColorWidth cw;
		color.serializeFloat(cw.m_color);
		cw.width = 1;
		int index = -1;

		int* indexPtr = m_hashedLines.find(cw);
		if (indexPtr)
		{
			index = *indexPtr;
		}
		else
		{
			index = m_sortedLines.size();
			m_sortedLines.expand();
			m_sortedIndices.expand();
			m_hashedLines.insert(cw, index);
		}
		btAssert(index >= 0);
		if (index >= 0)
		{
			btVector3FloatData from1, toX1;
			m_sortedIndices[index].push_back(m_sortedLines[index].size());
			from.serializeFloat(from1);
			m_sortedLines[index].push_back(from1);
			m_sortedIndices[index].push_back(m_sortedLines[index].size());
			to.serializeFloat(toX1);
			m_sortedLines[index].push_back(toX1);
		}
	}
}

void OpenCLDebugDrawer::drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color)
{
	drawSphere(PointOnB, 0.05, color);
	drawLine(PointOnB, PointOnB + normalOnB, color);
	btVector3 ncolor(0, 0, 0);
	drawLine(PointOnB, PointOnB + normalOnB * 0.5, ncolor);
}

void OpenCLDebugDrawer::reportErrorWarning(const char* warningString)
{

}

void OpenCLDebugDrawer::draw3dText(const btVector3& location, const char* textString)
{

}

void OpenCLDebugDrawer::setDebugMode(int debugMode)
{
	m_debugMode = debugMode;
}

int OpenCLDebugDrawer::getDebugMode() const
{
	return m_debugMode;
}

void OpenCLDebugDrawer::clearLines()
{
	m_hashedLines.clear();
	m_sortedIndices.clear();
	m_sortedLines.clear();
}

void OpenCLDebugDrawer::flushLines()
{

}