#pragma once

#include <LinearMath/btIDebugDraw.h>
#include <LinearMath/btAlignedObjectArray.h>
#include <LinearMath/btHashMap.h>

#include <CommonInterfaces/CommonGUIHelperInterface.h>

struct ColorWidth
{
	btVector3FloatData m_color;
	int width;
	int getHash() const
	{
		unsigned char r = (unsigned char)m_color.m_floats[0] * 255;
		unsigned char g = (unsigned char)m_color.m_floats[1] * 255;
		unsigned char b = (unsigned char)m_color.m_floats[2] * 255;
		unsigned char w = width;
		return r + (256 * g) + (256 * 256 * b) + (256 * 256 * 256 * w);
	}
	bool equals(const ColorWidth& other) const
	{
		bool same = ((width == other.width) && (m_color.m_floats[0] == other.m_color.m_floats[0]) &&
					 (m_color.m_floats[1] == other.m_color.m_floats[1]) &&
					 (m_color.m_floats[2] == other.m_color.m_floats[2]));
		return same;
	}
};

ATTRIBUTE_ALIGNED16(class)
OpenCLDebugDrawer : public btIDebugDraw
{
	GUIHelperInterface* m_guiHelper;
	int m_debugMode;

	btAlignedObjectArray<btAlignedObjectArray<unsigned int> > m_sortedIndices;
	btAlignedObjectArray<btAlignedObjectArray<btVector3FloatData> > m_sortedLines;
	btHashMap<ColorWidth, int> m_hashedLines;

public:
	virtual void drawDebugDrawerLines();
	
	OpenCLDebugDrawer(GUIHelperInterface * guiHelper);
	virtual ~OpenCLDebugDrawer();

	virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color);
	virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color);

	virtual void reportErrorWarning(const char* warningString);
	virtual void draw3dText(const btVector3& location, const char* textString);
	
	virtual void setDebugMode(int debugMode);
	virtual int getDebugMode() const;

	virtual void clearLines() override;
	virtual void flushLines();
};