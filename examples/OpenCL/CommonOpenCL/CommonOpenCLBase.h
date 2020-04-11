#ifndef COMMON_MULTI_BODY_SETUP_H
#define COMMON_MULTI_BODY_SETUP_H

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonWindowInterface.h"
#include "../CommonInterfaces/CommonCameraInterface.h"

#include "GpuDemoInternalData.h"
#include "Bullet3Common/b3Scalar.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"

struct CommonOpenCLBase : public CommonExampleInterface
{
	struct GUIHelperInterface* m_guiHelper;
	struct GpuDemoInternalData* m_clData;

	CommonOpenCLBase(GUIHelperInterface* helper)
		: m_guiHelper(helper),
		  m_clData(0)
	{
		m_clData = new GpuDemoInternalData();
	}

	virtual ~CommonOpenCLBase()
	{
		delete m_clData;
		m_clData = 0;
	}

	virtual void stepSimulation(float deltaTime)
	{
	}

	virtual void initCL(int preferredDeviceIndex, int preferredPlatformIndex)
	{
		//	void* glCtx=0;
		//	void* glDC = 0;

		int ciErrNum = 0;

		cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
		//if (gAllowCpuOpenCL)
		//	deviceType = CL_DEVICE_TYPE_ALL;

		//	if (useInterop)
		//	{
		//		m_data->m_clContext = b3OpenCLUtils::createContextFromType(deviceType, &ciErrNum, glCtx, glDC);
		//	} else
		{
			m_clData->m_clContext = b3OpenCLUtils::createContextFromType(deviceType, &ciErrNum, 0, 0, preferredDeviceIndex, preferredPlatformIndex, &m_clData->m_platformId);
		}

		oclCHECKERROR(ciErrNum, CL_SUCCESS);

		int numDev = b3OpenCLUtils::getNumDevices(m_clData->m_clContext);

		if (numDev > 0)
		{
			m_clData->m_clDevice = b3OpenCLUtils::getDevice(m_clData->m_clContext, 0);
			m_clData->m_clQueue = clCreateCommandQueue(m_clData->m_clContext, m_clData->m_clDevice, 0, &ciErrNum);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);

			b3OpenCLDeviceInfo info;
			b3OpenCLUtils::getDeviceInfo(m_clData->m_clDevice, &info);
			m_clData->m_clDeviceName = info.m_deviceName;
			m_clData->m_clInitialized = true;
		}
	}

	virtual void exitCL()
	{
		if (m_clData && m_clData->m_clInitialized)
		{
			clReleaseCommandQueue(m_clData->m_clQueue);
			clReleaseContext(m_clData->m_clContext);
			m_clData->m_clInitialized = false;
		}
	}

	virtual void renderScene()
	{
		if (m_guiHelper->getRenderInterface())
		{
			m_guiHelper->getRenderInterface()->renderScene();
		}
	}

	virtual void physicsDebugDraw(int debugDrawFlags)
	{
	}

	virtual bool keyboardCallback(int key, int state)
	{
		return false;  //don't handle this key
	}

	virtual bool mouseMoveCallback(float x, float y)
	{
		return false;
	}

	virtual bool pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)
	{
		/*if (m_dynamicsWorld == 0)
			return false;

		//btCollisionWorld::ClosestRayResultCallback rayCallback(rayFromWorld, rayToWorld);
		MyResultCallback rayCallback(rayFromWorld, rayToWorld);
		rayCallback.m_flags |= btTriangleRaycastCallback::kF_UseGjkConvexCastRaytest;
		m_data->m_dynamicsWorld->rayTest(rayFromWorld, rayToWorld, rayCallback);
		if (rayCallback.hasHit())
		{
			btVector3 pickPos = rayCallback.m_hitPointWorld;

			btRigidBody* body = (btRigidBody*)btRigidBody::upcast(rayCallback.m_collisionObject);
			if (body)
			{
				//other exclusions?
				if (!(body->isStaticObject() || body->isKinematicObject()))
				{
					m_data->m_pickedBody = body;
					m_data->m_savedActivationState = body->getActivationState();
					m_data->m_pickedBody->setActivationState(DISABLE_DEACTIVATION);
					//printf("pickPos=%f,%f,%f\n",pickPos.getX(),pickPos.getY(),pickPos.getZ());
					btVector3 localPivot = body->getCenterOfMassTransform().inverse() * pickPos;
					btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body, localPivot);
					m_data->m_dynamicsWorld->addConstraint(p2p, true);
					m_data->m_pickedConstraint = p2p;
					btScalar mousePickClamping = 30.f;
					p2p->m_setting.m_impulseClamp = mousePickClamping;
					//very weak constraint for picking
					p2p->m_setting.m_tau = 0.001f;
				}
			}
			else
			{
				btMultiBodyLinkCollider* multiCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(rayCallback.m_collisionObject);
				if (multiCol && multiCol->m_multiBody)
				{
					m_data->m_prevCanSleep = multiCol->m_multiBody->getCanSleep();
					multiCol->m_multiBody->setCanSleep(false);

					btVector3 pivotInA = multiCol->m_multiBody->worldPosToLocal(multiCol->m_link, pickPos);

					btMultiBodyPoint2Point* p2p = new btMultiBodyPoint2Point(multiCol->m_multiBody, multiCol->m_link, 0, pivotInA, pickPos);
					//if you add too much energy to the system, causing high angular velocities, simulation 'explodes'
					//see also http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=4&t=949
					//so we try to avoid it by clamping the maximum impulse (force) that the mouse pick can apply
					//it is not satisfying, hopefully we find a better solution (higher order integrator, using joint friction using a zero-velocity target motor with limited force etc?)
					btScalar scaling = 10;
					p2p->setMaxAppliedImpulse(2 * scaling);

					btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*)m_data->m_dynamicsWorld;
					world->addMultiBodyConstraint(p2p);
					m_data->m_pickingMultiBodyPoint2Point = p2p;
				}
			}

			//					pickObject(pickPos, rayCallback.m_collisionObject);
			m_data->m_oldPickingPos = rayToWorld;
			m_data->m_hitPos = pickPos;
			m_data->m_oldPickingDist = (pickPos - rayFromWorld).length();
			//					printf("hit !\n");
			//add p2p
		}*/
		return false;
	}

	virtual bool mouseButtonCallback(int button, int state, float x, float y)
	{
		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();

		if (!renderer)
		{
			b3Assert(0);
			return false;
		}

		CommonWindowInterface* window = m_guiHelper->getAppInterface()->m_window;

		if (state == 1)
		{
			if (button == 0 && (!window->isModifierKeyPressed(B3G_ALT) && !window->isModifierKeyPressed(B3G_CONTROL)))
			{
				b3Vector3 camPos;
				renderer->getActiveCamera()->getCameraPosition(camPos);

				/*b3Vector3 rayFrom = camPos;
				b3Vector3 rayTo = getRayTo(int(x), int(y));

				b3PickBody(rayFrom, rayTo);*/
			}
		}
		else
		{
			if (button == 0)
			{
				//				removePickingConstraint();
				//remove p2p
			}
		}

		printf("button=%d, state=%d\n",button,state);
		return false;
	}
};

#endif  //COMMON_MULTI_BODY_SETUP_H
