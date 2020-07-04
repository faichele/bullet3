#ifndef CONCAVE_SCENE_H
#define CONCAVE_SCENE_H

#include "GpuRigidBodyDemo.h"
#include "Bullet3Common/b3Vector3.h"

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonWindowInterface.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"

#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMatrix3x3.h"

#include "btOpenCLDebugDrawer.h"

class CommonExampleInterface* GPUConcaveSceneCreateFunc(struct CommonExampleOptions& options);

class ConcaveScene : public GpuRigidBodyDemo
{
public:
	ConcaveScene(struct GUIHelperInterface* helper) : GpuRigidBodyDemo(helper),
													  m_leftMouseButton(false),
													  m_middleMouseButton(false),
													  m_rightMouseButton(false),
													  m_wheelMultiplier(1.0f),
													  m_mouseMoveMultiplier(0.4f),
													  m_mouseXpos(0.f),
													  m_mouseYpos(0.f),
													  m_mouseInitialized(false)
	{
		m_guiHelper = helper;
		m_debugDrawer = new OpenCLDebugDrawer(m_guiHelper);
	}

	virtual ~ConcaveScene()
	{
		if (m_debugDrawer)
		{
			delete m_debugDrawer;
			m_debugDrawer = nullptr;
		}
	}

	GUIHelperInterface* m_guiHelper;
	OpenCLDebugDrawer* m_debugDrawer;

	/* extra stuff*/
	btVector3 m_cameraPosition;
	class btDiscreteDynamicsWorld* m_dynamicsWorld;
	btDiscreteDynamicsWorld* getDynamicsWorld()
	{
		return m_dynamicsWorld;
	}
	// btRigidBody* m_carChassis;
	// btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& worldTransform, btCollisionShape* colSape);

	bool m_useDefaultCamera;
	//----------------------------

	float m_cameraHeight;

	float m_minCameraDistance;
	float m_maxCameraDistance;

	bool m_leftMouseButton;
	bool m_middleMouseButton;
	bool m_rightMouseButton;
	float m_wheelMultiplier;
	float m_mouseMoveMultiplier;
	float m_mouseXpos;
	float m_mouseYpos;
	bool m_mouseInitialized;

	virtual void stepSimulation(float deltaTime);

	virtual void clientResetScene();

	virtual void displayCallback();

	virtual void specialKeyboard(int key, int x, int y);

	virtual void specialKeyboardUp(int key, int x, int y);

	virtual bool mouseMoveCallback(float x, float y);

	virtual bool mouseButtonCallback(int button, int state, float x, float y);

	virtual bool keyboardCallback(int key, int state);

	virtual void renderScene();

	virtual void physicsDebugDraw(int debugFlags);

	void initPhysics();
	void exitPhysics();

	virtual void resetCamera()
	{
		float dist = 500;
		float pitch = -32;
		float yaw = -45;
		float targetPos[3] = {0.0, 0.0, 25.0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

	void setupScene();

	virtual void createDynamicObjects(unsigned int arraySizeX = 100, unsigned int arraySizeY = 50, unsigned int arraySizeZ = 5, bool useInstancedCollisionShapes = true, float scale = 0.1);

	virtual void createConcaveMesh(const char* fileName, const b3Vector3& shift, const b3Vector3& scaling, bool ghostObject = false, float mass = 0.0f, int collisionMask = 0);
};

/*class ConcaveSphereScene : public ConcaveScene
{
public:
	ConcaveSphereScene() {}
	virtual ~ConcaveSphereScene() {}
	virtual const char* getName()
	{
		return "SphereTrimesh";
	}

	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new ConcaveSphereScene;
		return demo;
	}

	virtual void setupScene(const ConstructionInfo& ci);

	virtual void createDynamicObjects(const ConstructionInfo& ci);
};

class ConcaveCompoundScene : public ConcaveScene
{
public:
	ConcaveCompoundScene() {}
	virtual ~ConcaveCompoundScene() {}
	virtual const char* getName()
	{
		return "CompoundConcave";
	}

	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new ConcaveCompoundScene;
		return demo;
	}

	virtual void setupScene(const ConstructionInfo& ci);

	virtual void createDynamicObjects(const ConstructionInfo& ci);
};

class ConcaveCompound2Scene : public ConcaveCompoundScene
{
public:
	ConcaveCompound2Scene() {}
	virtual ~ConcaveCompound2Scene() {}
	virtual const char* getName()
	{
		return "GRBConcave2Compound";
	}

	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new ConcaveCompound2Scene;
		return demo;
	}
	virtual void createDynamicObjects(const ConstructionInfo& ci);
};
*/
#endif  //CONCAVE_SCENE_H
