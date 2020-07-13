#include "ConcaveScene.h"
#include "GpuRigidBodyDemo.h"
#include "OpenGLWindow/ShapeData.h"

#include "OpenGLWindow/GLInstancingRenderer.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "OpenGLWindow/OpenGLInclude.h"
#include "OpenGLWindow/GLInstanceRendererInternalData.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
#include "Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3Config.h"
#include "GpuRigidBodyDemoInternalData.h"
#include "../../Wavefront/tiny_obj_loader.h"
#include "Bullet3Common/b3Transform.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3ConvexUtility.h"

#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyData.h"

#include "Utils/b3BulletDefaultFileIO.h"

#include "OpenGLWindow/GLInstanceGraphicsShape.h"

#include <iostream>

#define CONCAVE_GAPX 5
#define CONCAVE_GAPY 5
#define CONCAVE_GAPZ 5

std::shared_ptr<GLInstanceGraphicsShape> createGraphicsShapeFromWavefrontObj(tinyobj::attrib_t& attrs, std::vector<tinyobj::shape_t>& shapes)
{
	b3AlignedObjectArray<GLInstanceVertex>* vertices = new b3AlignedObjectArray<GLInstanceVertex>;
	{
		//		int numVertices = obj->vertexCount;
		//	int numIndices = 0;
		b3AlignedObjectArray<int>* indicesPtr = new b3AlignedObjectArray<int>;

		for (int s = 0; s < shapes.size(); s++)
		{
			tinyobj::shape_t& shape = shapes[s];
			int faceCount = shape.mesh.indices.size();

			for (int f = 0; f < faceCount; f += 3)
			{
				//b3Vector3 normal(face.m_plane[0],face.m_plane[1],face.m_plane[2]);
				if (1)
				{
					b3Vector3 normal = b3MakeVector3(0, 1, 0);
					int vtxBaseIndex = vertices->size();

					indicesPtr->push_back(vtxBaseIndex);
					indicesPtr->push_back(vtxBaseIndex + 1);
					indicesPtr->push_back(vtxBaseIndex + 2);

					GLInstanceVertex vtx0;
					vtx0.xyzw[0] = attrs.vertices[(int)shape.mesh.indices[f].vertex_index * 3 + 0];
					vtx0.xyzw[1] = attrs.vertices[(int)shape.mesh.indices[f].vertex_index * 3 + 1];
					vtx0.xyzw[2] = attrs.vertices[(int)shape.mesh.indices[f].vertex_index * 3 + 2];
					vtx0.xyzw[3] = 0.f;

					vtx0.uv[0] = 0.5f;  //shape.mesh.positions[shape.mesh.indices[f]*3+2];?
					vtx0.uv[1] = 0.5f;

					GLInstanceVertex vtx1;
					vtx1.xyzw[0] = attrs.vertices[(int)shape.mesh.indices[f + 1].vertex_index * 3 + 0];
					vtx1.xyzw[1] = attrs.vertices[(int)shape.mesh.indices[f + 1].vertex_index * 3 + 1];
					vtx1.xyzw[2] = attrs.vertices[(int)shape.mesh.indices[f + 1].vertex_index * 3 + 2];
					vtx1.xyzw[3] = 0.f;
					vtx1.uv[0] = 0.5f;  //obj->textureList[face->vertex_index[1]]->e[0];
					vtx1.uv[1] = 0.5f;  //obj->textureList[face->vertex_index[1]]->e[1];

					GLInstanceVertex vtx2;
					vtx2.xyzw[0] = attrs.vertices[shape.mesh.indices[f + 2].vertex_index * 3 + 0];
					vtx2.xyzw[1] = attrs.vertices[shape.mesh.indices[f + 2].vertex_index * 3 + 1];
					vtx2.xyzw[2] = attrs.vertices[shape.mesh.indices[f + 2].vertex_index * 3 + 2];
					vtx2.xyzw[3] = 0.f;
					vtx2.uv[0] = 0.5f;
					vtx2.uv[1] = 0.5f;

					b3Vector3 v0 = b3MakeVector3(vtx0.xyzw[0], vtx0.xyzw[1], vtx0.xyzw[2]);
					b3Vector3 v1 = b3MakeVector3(vtx1.xyzw[0], vtx1.xyzw[1], vtx1.xyzw[2]);
					b3Vector3 v2 = b3MakeVector3(vtx2.xyzw[0], vtx2.xyzw[1], vtx2.xyzw[2]);

					normal = (v1 - v0).cross(v2 - v0);
					normal.normalize();
					vtx0.normal[0] = normal[0];
					vtx0.normal[1] = normal[1];
					vtx0.normal[2] = normal[2];
					vtx1.normal[0] = normal[0];
					vtx1.normal[1] = normal[1];
					vtx1.normal[2] = normal[2];
					vtx2.normal[0] = normal[0];
					vtx2.normal[1] = normal[1];
					vtx2.normal[2] = normal[2];
					vertices->push_back(vtx0);
					vertices->push_back(vtx1);
					vertices->push_back(vtx2);
				}
			}
		}

		std::shared_ptr<GLInstanceGraphicsShape> gfxShape;
		gfxShape.reset(new GLInstanceGraphicsShape);
		gfxShape->m_vertices = vertices;
		gfxShape->m_numvertices = vertices->size();
		gfxShape->m_indices = indicesPtr;
		gfxShape->m_numIndices = indicesPtr->size();
		for (int i = 0; i < 4; i++)
			gfxShape->m_scaling[i] = 1;  //bake the scaling into the vertices

		return gfxShape;
	}
}

void ConcaveScene::clientResetScene()
{
	exitPhysics();
	initPhysics();
}

void ConcaveScene::displayCallback(void)
{
	//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//renderme();

	//optional but useful: debug drawing
	/*if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();*/

	//	glFlush();
	//	glutSwapBuffers();
}

void ConcaveScene::stepSimulation(float deltaTime)
{
	GpuRigidBodyDemo::stepSimulation(deltaTime);
}

void ConcaveScene::specialKeyboard(int key, int x, int y)
{
}

void ConcaveScene::specialKeyboardUp(int key, int x, int y)
{
}

bool ConcaveScene::keyboardCallback(int key, int state)
{
	return false;
}

bool ConcaveScene::mouseMoveCallback(float x, float y)
{
	CommonCameraInterface* camera = m_guiHelper->getRenderInterface()->getActiveCamera();

	if (camera)
	{
		bool isAltPressed = m_window->isModifierKeyPressed(B3G_ALT);
		bool isControlPressed = m_window->isModifierKeyPressed(B3G_CONTROL);

		if (isAltPressed || isControlPressed)
		{
			float xDelta = x - m_mouseXpos;
			float yDelta = y - m_mouseYpos;
			float cameraDistance = camera->getCameraDistance();
			float pitch = camera->getCameraPitch();
			float yaw = camera->getCameraYaw();

			float targPos[3];
			float camPos[3];

			camera->getCameraTargetPosition(targPos);
			camera->getCameraPosition(camPos);

			b3Vector3 cameraPosition = b3MakeVector3(b3Scalar(camPos[0]),
													 b3Scalar(camPos[1]),
													 b3Scalar(camPos[2]));

			b3Vector3 cameraTargetPosition = b3MakeVector3(b3Scalar(targPos[0]),
														   b3Scalar(targPos[1]),
														   b3Scalar(targPos[2]));
			b3Vector3 cameraUp = b3MakeVector3(0, 0, 0);
			cameraUp[camera->getCameraUpAxis()] = 1.f;

			if (m_leftMouseButton)
			{
				//			if (b3Fabs(xDelta)>b3Fabs(yDelta))
				//			{
				pitch -= yDelta * m_mouseMoveMultiplier;
				//			} else
				//			{
				yaw -= xDelta * m_mouseMoveMultiplier;
				//			}
			}

			if (m_middleMouseButton)
			{
				cameraTargetPosition += cameraUp * yDelta * m_mouseMoveMultiplier * 0.01;

				b3Vector3 fwd = cameraTargetPosition - cameraPosition;
				b3Vector3 side = cameraUp.cross(fwd);
				side.normalize();
				cameraTargetPosition += side * xDelta * m_mouseMoveMultiplier * 0.01;
			}
			if (m_rightMouseButton)
			{
				cameraDistance -= xDelta * m_mouseMoveMultiplier * 0.01f;
				cameraDistance -= yDelta * m_mouseMoveMultiplier * 0.01f;
				if (cameraDistance < 1)
					cameraDistance = 1;
				if (cameraDistance > 1000)
					cameraDistance = 1000;
			}
			camera->setCameraDistance(cameraDistance);
			camera->setCameraPitch(pitch);
			camera->setCameraYaw(yaw);
			camera->setCameraTargetPosition(cameraTargetPosition[0], cameraTargetPosition[1], cameraTargetPosition[2]);
		}
	}
	m_mouseXpos = x;
	m_mouseYpos = y;
	m_mouseInitialized = true;

	return true;
}

bool ConcaveScene::mouseButtonCallback(int button, int state, float x, float y)
{
	if (button == 0)
		m_leftMouseButton = (state == 1);
	if (button == 1)
		m_middleMouseButton = (state == 1);

	if (button == 2)
		m_rightMouseButton = (state == 1);

	m_mouseXpos = x;
	m_mouseYpos = y;
	m_mouseInitialized = true;

	return true;
}

void ConcaveScene::renderScene()
{
	std::cout << "Graphics instances in scene: " << m_instancingRenderer->getTotalNumInstances() << std::endl;
	std::cout << "Rigid objects in scene     : " << m_data->m_rigidBodyPipeline->getNumBodies() << std::endl;
	physicsDebugDraw(0);
	GpuRigidBodyDemo::renderScene();
}

void ConcaveScene::physicsDebugDraw(int debugFlags)
{
	unsigned int numContactPoints = m_data->m_rigidBodyPipeline->getNumContacts();
	std::cout << "Contact point count: " << numContactPoints << std::endl;

	btVector3 defaultContactColor(0.8, 0.4, 0.4);
	btVector3 ghostContactColor(0.4, 0.8, 0.4);

	btVector3 triangleColor(0.2, 0.8, 0.2);
	btVector3 ghostObjectColor(0.8, 0.8, 0.2);

	m_debugDrawer->clearLines();

	if (numContactPoints > 0)
	{
		const b3Contact4* contactPoints = m_data->m_rigidBodyPipeline->getContacts();
		const b3AlignedObjectArray<int>& collisionFlags = m_data->m_rigidBodyPipeline->getCollisionFlags();
		for (unsigned int k = 0; k < numContactPoints; ++k)
		{
			for (unsigned int l = 0; l < contactPoints[k].getNPoints(); ++l)
			{
				btVector3 posB(contactPoints[k].m_worldPosB[l].x, contactPoints[k].m_worldPosB[l].y, contactPoints[k].m_worldPosB[l].z);
				btVector3 normalOnB(contactPoints[k].m_worldNormalOnB.x, contactPoints[k].m_worldNormalOnB.y, contactPoints[k].m_worldNormalOnB.z);
				float dist = contactPoints[k].getPenetration(l);

				bool ghostContact = false;
				if (contactPoints[k].getBodyA() < collisionFlags.size() && contactPoints[k].getBodyB() < collisionFlags.size())
				{
					if ((collisionFlags[contactPoints[k].getBodyA()] & CF_GHOST_OBJECT) ||
						(collisionFlags[contactPoints[k].getBodyA()] & CF_GHOST_OBJECT))
						ghostContact = true;
				}

				// std::cout << "Draw contact point at: (" << posB.x() << ", " << posB.y() << ", " << posB.z() << "); normal: (" << normalOnB.x() << ", " << normalOnB.y() << ", " << normalOnB.z() << "); distance: " << dist << std::endl;

				if (ghostContact)
					m_debugDrawer->drawContactPoint(posB, normalOnB, dist, 1, ghostContactColor);
				else
					m_debugDrawer->drawContactPoint(posB, normalOnB, dist, 1, defaultContactColor);
			}
		}
	}

	int numRigidBodies = m_data->m_rigidBodyPipeline->getNumBodies();
	std::cout << "Rigid body count: " << numRigidBodies << std::endl;
	auto rigidBodies = m_data->m_np->getBodiesCpu();
	const b3AlignedObjectArray<b3RigidBodyPushPullBehavior>& pushPullBehaviors = m_data->m_rigidBodyPipeline->getPushPullBehaviors();
	std::cout << "Push-pull behaviors count: " << pushPullBehaviors.size() << std::endl;
	for (size_t m = 0; m < pushPullBehaviors.size(); ++m)
	{
		std::cout << "Debug draw push-pull behavior: " << m << std::endl;
		if (pushPullBehaviors[m].m_bodyID < numRigidBodies && pushPullBehaviors[m].m_bodyID >= 0)
		{
			btVector3 dirStart;           //(rigidBodies[pushPullBehaviors[m].m_bodyID].m_pos.x, rigidBodies[pushPullBehaviors[m].m_bodyID].m_pos.y, rigidBodies[pushPullBehaviors[m].m_bodyID].m_pos.z);
			btVector3 dirEnd = dirStart;  //+ btVector3(pushPullBehaviors[m].m_linearVel.x, pushPullBehaviors[m].m_linearVel.y, pushPullBehaviors[m].m_linearVel.z);
			
			m_debugDrawer->drawSphere(dirStart, 0.08, ghostObjectColor);
			m_debugDrawer->drawLine(dirStart, dirEnd, ghostObjectColor);

			btQuaternion bodyOrientation;
			btTransform endTf;
			endTf.setOrigin(dirEnd);
			endTf.setRotation(bodyOrientation);
			m_debugDrawer->drawCone(0.08, 0.08, 2, endTf, ghostObjectColor);

			std::cout << " With rigid body ID: " << pushPullBehaviors[m].m_bodyID << " drawn from (" << dirStart.x() << "," << dirStart.y() << "," << dirStart.z() << ") to (" << dirEnd.x() << "," << dirEnd.y() << "," << dirEnd.z() << ")" << std::endl;
		}
	}

	size_t numConcaveMeshes = m_data->m_np->getNumConcaveMeshes();
	if (numConcaveMeshes > 0)
	{
		b3AlignedObjectArray<int> concaveMeshCollidableIDs;
		m_data->m_np->getConcaveMeshCollidableIDs(concaveMeshCollidableIDs);
		if (numConcaveMeshes == concaveMeshCollidableIDs.size())
		{
			for (size_t k = 0; k < numConcaveMeshes; ++k)
			{
				bool ghostObject = (std::find(m_ghostObjectColIndices.begin(), m_ghostObjectColIndices.end(), concaveMeshCollidableIDs[k]) != m_ghostObjectColIndices.end());
				b3AlignedObjectArray<b3Vector3> meshVertices;
				b3AlignedObjectArray<int> meshIndices;
				if (m_data->m_np->getMeshVertices(concaveMeshCollidableIDs[k], meshVertices) &&
					m_data->m_np->getMeshIndices(concaveMeshCollidableIDs[k], meshIndices))
				{
					// Triangle meshes: index step 3
					for (int m = 0; m < meshIndices.size(); m += 3)
					{
						b3Vector3& v1 = meshVertices[meshIndices[m]];
						b3Vector3& v2 = meshVertices[meshIndices[m + 1]];
						b3Vector3& v3 = meshVertices[meshIndices[m + 2]];

						if (!ghostObject)
							m_debugDrawer->drawTriangle(btVector3(v1.x, v1.y, v1.z), btVector3(v2.x, v2.y, v2.z), btVector3(v3.x, v3.y, v3.z), triangleColor, 1.);
						else
							m_debugDrawer->drawTriangle(btVector3(v1.x, v1.y, v1.z), btVector3(v2.x, v2.y, v2.z), btVector3(v3.x, v3.y, v3.z), ghostObjectColor, 1.);
					}
				}
			}
		}
	}
	m_debugDrawer->drawDebugDrawerLines();
}

void ConcaveScene::initPhysics()
{
	GpuRigidBodyDemo::initPhysics();
}

void ConcaveScene::exitPhysics()
{
	GpuRigidBodyDemo::exitPhysics();
}

void ConcaveScene::setupScene()
{
	const char* fileName = "cube.obj";  //"samurai_monastry.obj";

	int graphicsId = -1, physicsId = -1;

	b3Mat3x3 tmp;
	tmp.setEulerZYX(0, 0, 0);
	b3Quaternion orientation_plane;
	tmp.getRotation(orientation_plane);
	b3Vector3 position_plane = b3MakeVector3(0, -10, 0);
	b3Vector4 scaling_plane = b3MakeVector4(250, 5, 250, 1);
	createConcaveMesh(graphicsId, physicsId, fileName, position_plane, orientation_plane, scaling_plane);

	const char* fileName_conveyor1 = "foerderband_1.obj";
	const char* fileName_conveyor2 = "foerderband_2.obj";
	const char* fileName_conveyor3 = "foerderband_3.obj";
	const char* fileName_conveyor4 = "foerderband_4.obj";

	const char* fileName_conveyor1_ghost = "foerderband_1_ghost.obj";
	const char* fileName_conveyor2_ghost = "foerderband_2_ghost.obj";
	const char* fileName_conveyor3_ghost = "foerderband_3_ghost.obj";
	const char* fileName_conveyor4_ghost = "foerderband_4_ghost.obj";

	b3Quaternion orientation(0, 0, 0, 1);
	b3Vector3 position1 = b3MakeVector3(0, 5, 7.5);
	b3Vector3 position2 = b3MakeVector3(0, 5, 10);
	b3Vector3 position3 = b3MakeVector3(0, 5, 10);
	b3Vector3 position4 = b3MakeVector3(2, 5, 10);

	b3Vector4 scaling = b3MakeVector4(1, 1, 1, 1);

	createConcaveMesh(graphicsId, physicsId, fileName_conveyor1, position1, orientation, scaling);
	m_rigidBodyColIndices.push_back(physicsId);
	createConcaveMesh(graphicsId, physicsId, fileName_conveyor2, position2, orientation, scaling);
	m_rigidBodyColIndices.push_back(physicsId);
	createConcaveMesh(graphicsId, physicsId, fileName_conveyor3, position3, orientation, scaling);
	m_rigidBodyColIndices.push_back(physicsId);
	createConcaveMesh(graphicsId, physicsId, fileName_conveyor4, position4, orientation, scaling);
	m_rigidBodyColIndices.push_back(physicsId);

	if (createConcaveMesh(graphicsId, physicsId, fileName_conveyor1_ghost, position1, orientation, scaling, true, 0.0f, 1))
	{
		b3Vector3 trVel1 = b3MakeVector3(0, 0, 1);
		b3Vector3 rotVel1 = b3MakeVector3(0, 0, 0);
		m_data->m_rigidBodyPipeline->setPhysicsInstancePushPullBehavior(physicsId, trVel1, rotVel1);
		m_ghostObjectColIndices.push_back(physicsId);
	}
	if (createConcaveMesh(graphicsId, physicsId, fileName_conveyor2_ghost, position2, orientation, scaling, true, 0.0f, 2))
	{
		b3Vector3 trVel2 = b3MakeVector3(0, 0, 1.5);
		b3Vector3 rotVel2 = b3MakeVector3(0, 0, 0);
		m_data->m_rigidBodyPipeline->setPhysicsInstancePushPullBehavior(physicsId, trVel2, rotVel2);
		m_ghostObjectColIndices.push_back(physicsId);
	}
	if (createConcaveMesh(graphicsId, physicsId, fileName_conveyor3_ghost, position3, orientation, scaling, true, 0.0f, 3))
	{
		b3Vector3 trVel3 = b3MakeVector3(0, 0, 3.0);
		b3Vector3 rotVel3 = b3MakeVector3(0, 0, 0);
		m_data->m_rigidBodyPipeline->setPhysicsInstancePushPullBehavior(physicsId, trVel3, rotVel3);
		m_ghostObjectColIndices.push_back(physicsId);
	}
	if (createConcaveMesh(graphicsId, physicsId, fileName_conveyor4_ghost, position4, orientation, scaling, true, 0.0f, 4))
	{
		b3Vector3 trVel4 = b3MakeVector3(0, 0, 2.5);
		b3Vector3 rotVel4 = b3MakeVector3(0, 0, 0);
		m_data->m_rigidBodyPipeline->setPhysicsInstancePushPullBehavior(physicsId, trVel4, rotVel4);
		m_ghostObjectColIndices.push_back(physicsId);
	}

	// Y = up
	b3Vector3 objects_origin = b3MakeVector3(10, 40, 30);
	createDynamicObjects(objects_origin, 1, 1, 10, true, 0.75f);

	m_data->m_rigidBodyPipeline->writeAllInstancesToGpu();

	char msg[1024];
	int numInstances = m_data->m_rigidBodyPipeline->getNumBodies();
	sprintf(msg, "Num objects = %d", numInstances);
}

bool ConcaveScene::createConcaveMesh(int& graphicsId, int& physicsId, const char* fileName, const b3Vector3& position, const b3Quaternion& orientation, const b3Vector3& scaling, bool ghostObject, float mass, int collisionMask)
{
	char relativeFileName[1024];
	const char* prefix[] = {"./data/", "../data/", "../../data/", "../../../data/", "../../../../data/"};
	int prefixIndex = -1;
	{
		int numPrefixes = sizeof(prefix) / sizeof(char*);

		for (int i = 0; i < numPrefixes; i++)
		{
			FILE* f = 0;
			sprintf(relativeFileName, "%s%s", prefix[i], fileName);
			f = fopen(relativeFileName, "r");
			if (f)
			{
				fclose(f);
				prefixIndex = i;
				break;
			}
		}
	}

	if (prefixIndex < 0)
		return false;

	int index = 10;

	{
		std::vector<tinyobj::shape_t> shapes;
		tinyobj::attrib_t attribs;
		b3BulletDefaultFileIO fileIO;
		std::string err = tinyobj::LoadObj(attribs, shapes, relativeFileName, prefix[prefixIndex], &fileIO);

		std::shared_ptr<GLInstanceGraphicsShape> shape = createGraphicsShapeFromWavefrontObj(attribs, shapes);

		b3AlignedObjectArray<b3Vector3> verts;
		b3AlignedObjectArray<b3Vector3> untransformed_verts;
		for (int i = 0; i < shape->m_numvertices; i++)
		{
			b3Vector3 vtx_untransformed = b3MakeVector3(shape->m_vertices->at(i).xyzw[0],
														shape->m_vertices->at(i).xyzw[1],
														shape->m_vertices->at(i).xyzw[2]);
			untransformed_verts.push_back(vtx_untransformed * scaling);

			// Setting position of the mesh later instead of shifting the individual vertices?
			for (int j = 0; j < 3; j++)
				shape->m_vertices->at(i).xyzw[j] += position[j];

			b3Vector3 vtx = b3MakeVector3(shape->m_vertices->at(i).xyzw[0],
										  shape->m_vertices->at(i).xyzw[1],
										  shape->m_vertices->at(i).xyzw[2]);
			verts.push_back(vtx * scaling);
		}

		// TODO: Cordinate conventions for (static/unmoving) collision geometries: Assume origin-centered?
		// Just take the vertex coordinates as-is? Explicit flag to add position as offset per vertex?
		int colIndex = -1;
		if (ghostObject)
			colIndex = m_data->m_np->registerConcaveMesh(&/*untransformed_verts*/ verts, shape->m_indices, b3MakeVector3(1, 1, 1));
		else
			colIndex = m_data->m_np->registerConcaveMesh(&verts, shape->m_indices, b3MakeVector3(1, 1, 1));

		{
			/*int strideInBytes = 9 * sizeof(float);
			int numVertices = sizeof(cube_vertices) / strideInBytes;
			int numIndices = sizeof(cube_indices) / sizeof(int);

            int shapeId_cube = m_guiHelper->getRenderInterface()->registerShape(&cube_vertices[0], numVertices, cube_indices, numIndices);*/
			//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);

			int shapeId_shape = m_guiHelper->getRenderInterface()->registerShape(&shape->m_vertices->at(0).xyzw[0], shape->m_numvertices, &shape->m_indices->at(0), shape->m_numIndices);

			b3Vector4 color = b3MakeVector4(0.3, 0.3, 0.8, 1.0);
			if (ghostObject)
				color = b3MakeVector4(0.2, 0.8, 0.2, 0.33);

			{
				// This is solved better by directly setting the object's position instead of offsetting the mesh vertices by the given shift!
				b3Vector3 mesh_position = b3MakeVector3(0, 0, 0);
				int graphics_id = m_instancingRenderer->registerGraphicsInstance(shapeId_shape, position, orientation, color, scaling);
				graphicsId = graphics_id;

				int physics_id = -1;

				if (ghostObject)
					physics_id = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass, mesh_position, orientation, colIndex, index, false, b3CollisionFlags::CF_GHOST_OBJECT, collisionMask);
				else
					physics_id = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass, mesh_position, orientation, colIndex, index, false);

				physicsId = physics_id;

				index++;
			}

			delete shape->m_indices;
			shape->m_indices = nullptr;
			shape->m_numIndices = 0;
			delete shape->m_vertices;
			shape->m_vertices = nullptr;
			shape->m_numvertices = 0;

			return true;
		}
	}
	return false;
}

void ConcaveScene::createDynamicObjects(const b3Vector3& objects_origin, unsigned int arraySizeX, unsigned int arraySizeY, unsigned int arraySizeZ, bool useInstancedCollisionShapes, float scale)
{
	int strideInBytes = 9 * sizeof(float);
	int numVertices = sizeof(cube_vertices) / strideInBytes;
	int numIndices = sizeof(cube_indices) / sizeof(int);

	int shapeId = m_instancingRenderer->registerShape(&cube_vertices[0], numVertices, cube_indices, numIndices);
	int group = 1;
	int mask = 1;

	int index = 0;

	int curColor = 0;
	b3Vector4 colors[4] =
		{
			b3MakeVector4(1, 1, 1, 1),
			b3MakeVector4(1, 1, 0.3, 1),
			b3MakeVector4(0.3, 1, 1, 1),
			b3MakeVector4(0.3, 0.3, 1, 1),
		};

	b3ConvexUtility* utilPtr = new b3ConvexUtility();
	b3Vector4 scaling = b3MakeVector4(1, 1, 1, 1);

	if (scale > 0.0f && scale <= 1.0f)
		scaling = b3MakeVector4(scale, scale, scale, scale);

	{
		b3AlignedObjectArray<b3Vector3> verts;

		unsigned char* vts = (unsigned char*)cube_vertices;
		for (int i = 0; i < numVertices; i++)
		{
			float* vertex = (float*)&vts[i * strideInBytes];
			verts.push_back(b3MakeVector3(vertex[0] * scaling[0], vertex[1] * scaling[1], vertex[2] * scaling[2]));
		}

		bool merge = true;
		if (numVertices)
		{
			utilPtr->initializePolyhedralFeatures(&verts[0], verts.size(), merge);
		}
	}

	//		int colIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);

	int colIndex = -1;
	if (useInstancedCollisionShapes)
		colIndex = m_data->m_np->registerConvexHullShape(utilPtr);

	float mass = 1;

	for (int k = 0; k < 10; ++k)
	{
		b3Vector3 position = b3MakeVector3(objects_origin.x,
										   objects_origin.y,
										   objects_origin.z - k * 3);
		b3Quaternion orn(0, 0, 0, 1);

		b3Vector4 color = colors[curColor];
		curColor++;
		curColor &= 3;

		int id = m_instancingRenderer->registerGraphicsInstance(shapeId, position, orn, color, scaling);
		int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass, position, orn, colIndex, index, false);

		index++;
	}
	/*for (int i = 0; i < arraySizeX; i++)
	{
		for (int j = 0; j < arraySizeY; j++)
		{
			for (int k = 0; k < arraySizeZ; k++)
			{
				if (!useInstancedCollisionShapes)
					colIndex = m_data->m_np->registerConvexHullShape(utilPtr);

				float mass = 1;

				b3Vector3 position = b3MakeVector3(objects_origin.x + (-(arraySizeX / 2) * CONCAVE_GAPX + i * CONCAVE_GAPX),
												   objects_origin.y + 23 + j * CONCAVE_GAPY,
												   objects_origin.z + (-(arraySizeZ / 2) * CONCAVE_GAPZ + k * CONCAVE_GAPZ));
				b3Quaternion orn(0, 0, 0, 1);

				b3Vector4 color = colors[curColor];
				curColor++;
				curColor &= 3;

				int id = m_instancingRenderer->registerGraphicsInstance(shapeId, position, orn, color, scaling);
				int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass, position, orn, colIndex, index, false);

				index++;
			}
		}
	}*/
}

/*
void ConcaveScene::setupScene(const ConstructionInfo& ci)
{
	if (1)
	{
		//char* fileName = "slopedPlane100.obj";
		//char* fileName = "plane100.obj";
		//	char* fileName = "plane100.obj";

		//char* fileName = "teddy.obj";//"plane.obj";
		//	char* fileName = "sponza_closed.obj";//"plane.obj";
		//char* fileName = "leoTest1.obj";
		const char* fileName = "samurai_monastry.obj";
		//	char* fileName = "teddy2_VHACD_CHs.obj";

		b3Vector3 shift1 = b3MakeVector3(0, 0, 0);  //0,230,80);//150,-100,-120);

		b3Vector4 scaling = b3MakeVector4(10, 10, 10, 1);

		//	createConcaveMesh(ci,"plane100.obj",shift1,scaling);
		//createConcaveMesh(ci,"plane100.obj",shift,scaling);

		//	b3Vector3 shift2(0,0,0);//0,230,80);//150,-100,-120);
		//	createConcaveMesh(ci,"teddy.obj",shift2,scaling);

		//	b3Vector3 shift3(130,-150,-75);//0,230,80);//150,-100,-120);
		//	createConcaveMesh(ci,"leoTest1.obj",shift3,scaling);
		createConcaveMesh(ci, fileName, shift1, scaling);
	}
	else
	{
		int strideInBytes = 9 * sizeof(float);
		int numVertices = sizeof(cube_vertices) / strideInBytes;
		int numIndices = sizeof(cube_indices) / sizeof(int);
		int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0], numVertices, cube_indices, numIndices);
		int group = 1;
		int mask = 1;
		int index = 0;
		{
			b3Vector4 scaling = b3MakeVector4(400, 1., 400, 1);
			int colIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0], strideInBytes, numVertices, scaling);
			b3Vector3 position = b3MakeVector3(0, -2, 0);
			b3Quaternion orn(0, 0, 0, 1);

			b3Vector4 color = b3MakeVector4(0, 0, 1, 1);

			int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId, position, orn, color, scaling);
			int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(0.f, position, orn, colIndex, index, false);
		}
	}

	createDynamicObjects(ci);

	m_data->m_rigidBodyPipeline->writeAllInstancesToGpu();

	float camPos[4] = {0, 0, 0, 0};  //65.5,4.5,65.5,0};
	//float camPos[4]={1,12.5,1.5,0};
	m_instancingRenderer->setCameraPitch(45);
	m_instancingRenderer->setCameraTargetPosition(camPos);
	m_instancingRenderer->setCameraDistance(355);
	char msg[1024];
	int numInstances = m_data->m_rigidBodyPipeline->getNumBodies();
	sprintf(msg, "Num objects = %d", numInstances);
	if (ci.m_gui)
		ci.m_gui->setStatusBarMessage(msg, true);
}

*/

/*void ConcaveCompoundScene::setupScene(const ConstructionInfo& ci)
{
	ConcaveScene::setupScene(ci);

	float camPos[4] = {0, 50, 0, 0};  //65.5,4.5,65.5,0};
	//float camPos[4]={1,12.5,1.5,0};
	m_instancingRenderer->setCameraPitch(45);
	m_instancingRenderer->setCameraTargetPosition(camPos);
	m_instancingRenderer->setCameraDistance(40);
}

void ConcaveCompound2Scene::createDynamicObjects(const ConstructionInfo& ci)
{
	const char* fileName = "teddy2_VHACD_CHs.obj";
	//char* fileName = "cube_offset.obj";

	b3Vector3 shift = b3MakeVector3(0, 0, 0);  //0,230,80);//150,-100,-120);
	b3Vector4 scaling = b3MakeVector4(1, 1, 1, 1);
	const char* prefix[] = {"./data/", "../data/", "../../data/", "../../../data/", "../../../../data/"};
	int prefixIndex = -1;

	char relativeFileName[1024];
	{
		int numPrefixes = sizeof(prefix) / sizeof(char*);

		for (int i = 0; i < numPrefixes; i++)
		{
			sprintf(relativeFileName, "%s%s", prefix[i], fileName);
			FILE* f = 0;
			f = fopen(relativeFileName, "r");
			if (f)
			{
				prefixIndex = i;
				fclose(f);
				break;
			}
		}
	}

	if (prefixIndex < 0)
		return;

	std::vector<tinyobj::shape_t> shapes;
	std::string err = tinyobj::LoadObj(shapes, relativeFileName, prefix[prefixIndex]);

	if (shapes.size() > 0)
	{
		int strideInBytes = 9 * sizeof(float);

		b3AlignedObjectArray<GLInstanceVertex> vertexArray;
		b3AlignedObjectArray<int> indexArray;

		//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
		int group = 1;
		int mask = 1;
		int index = 0;
		int colIndex = 0;

		b3AlignedObjectArray<GLInstanceVertex> vertices;
		int stride2 = sizeof(GLInstanceVertex);
		b3Assert(stride2 == strideInBytes);

		{
			b3AlignedObjectArray<b3GpuChildShape> childShapes;

			int numChildShapes = shapes.size();

			for (int i = 0; i < numChildShapes; i++)
			//			int i=4;
			{
				tinyobj::shape_t& shape = shapes[i];

				int numVertices = shape.mesh.positions.size() / 3;
				int numFaces = shape.mesh.indices.size() / 3;

				//for now, only support polyhedral child shapes
				b3GpuChildShape child;

				b3Vector3 pos = b3MakeVector3(0, 0, 0);
				b3Quaternion orn(0, 0, 0, 1);
				for (int v = 0; v < 4; v++)
				{
					child.m_childPosition[v] = pos[v];
					child.m_childOrientation[v] = orn[v];
				}

				b3Transform tr;
				tr.setIdentity();
				tr.setOrigin(pos);
				tr.setRotation(orn);

				int baseIndex = vertexArray.size();

				for (int f = 0; f < numFaces; f++)
				{
					for (int i = 0; i < 3; i++)
					{
						indexArray.push_back(baseIndex + shape.mesh.indices[f * 3 + i]);
					}
				}

				b3Vector3 center = b3MakeVector3(0, 0, 0);

				b3AlignedObjectArray<GLInstanceVertex> tmpVertices;
				//add transformed graphics vertices and indices
				b3Vector3 myScaling = b3MakeVector3(50, 50, 50);  //300,300,300);
				for (int v = 0; v < numVertices; v++)
				{
					GLInstanceVertex vert;

					vert.uv[0] = 0.5f;
					vert.uv[1] = 0.5f;
					vert.normal[0] = 0.f;
					vert.normal[1] = 1.f;
					vert.normal[2] = 0.f;
					b3Vector3 vertPos;
					vertPos[0] = shape.mesh.positions[v * 3 + 0] * myScaling[0];
					vertPos[1] = shape.mesh.positions[v * 3 + 1] * myScaling[1];
					vertPos[2] = shape.mesh.positions[v * 3 + 2] * myScaling[2];
					vertPos[3] = 0.f;
					center += vertPos;
				}

				center /= numVertices;

				for (int v = 0; v < numVertices; v++)
				{
					GLInstanceVertex vert;
					vert.uv[0] = 0.5f;
					vert.uv[1] = 0.5f;
					vert.normal[0] = 0.f;
					vert.normal[1] = 1.f;
					vert.normal[2] = 0.f;
					b3Vector3 vertPos;
					vertPos[0] = shape.mesh.positions[v * 3 + 0] * myScaling[0];
					vertPos[1] = shape.mesh.positions[v * 3 + 1] * myScaling[1];
					vertPos[2] = shape.mesh.positions[v * 3 + 2] * myScaling[2];
					vertPos[3] = 0.f;
					//				vertPos-=center;
					vert.xyzw[0] = vertPos[0];
					vert.xyzw[1] = vertPos[1];
					vert.xyzw[2] = vertPos[2];

					tmpVertices.push_back(vert);
					b3Vector3 newPos = tr * vertPos;
					vert.xyzw[0] = newPos[0];
					vert.xyzw[1] = newPos[1];
					vert.xyzw[2] = newPos[2];
					vert.xyzw[3] = 0.f;
					vertexArray.push_back(vert);
				}

				int childColIndex = m_data->m_np->registerConvexHullShape(&tmpVertices[0].xyzw[0], strideInBytes, numVertices, scaling);
				child.m_shapeIndex = childColIndex;
				childShapes.push_back(child);
				colIndex = childColIndex;
			}
			colIndex = m_data->m_np->registerCompoundShape(&childShapes);
		}

		//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
		int shapeId = ci.m_instancingRenderer->registerShape(&vertexArray[0].xyzw[0], vertexArray.size(), &indexArray[0], indexArray.size());

		b3Vector4 colors[4] =
			{
				b3MakeVector4(1, 0, 0, 1),
				b3MakeVector4(0, 1, 0, 1),
				b3MakeVector4(0, 0, 1, 1),
				b3MakeVector4(0, 1, 1, 1),
			};

		int curColor = 0;
		for (int i = 0; i < 1; i++)  //ci.arraySizeX;i++)
		{
			for (int j = 0; j < 4; j++)
			{
				//		for (int k=0;k<ci.arraySizeZ;k++)
				int k = 0;
				{
					float mass = 1;  //j==0? 0.f : 1.f;

					//b3Vector3 position(i*10*ci.gapX,j*ci.gapY,k*10*ci.gapZ);
					b3Vector3 position = b3MakeVector3(i * 10 * ci.gapX, 10 + j * 10 * ci.gapY, k * 10 * ci.gapZ);

					//	b3Quaternion orn(0,0,0,1);
					b3Quaternion orn(b3MakeVector3(0, 0, 1), 1.8);

					b3Vector4 color = colors[curColor];
					curColor++;
					curColor &= 3;
					b3Vector4 scaling = b3MakeVector4(1, 1, 1, 1);
					int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId, position, orn, color, scaling);
					int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass, position, orn, colIndex, index, false);

					index++;
				}
			}
		}
	}
}

void ConcaveCompoundScene::createDynamicObjects(const ConstructionInfo& ci)
{
	int strideInBytes = 9 * sizeof(float);
	int numVertices = sizeof(cube_vertices) / strideInBytes;
	int numIndices = sizeof(cube_indices) / sizeof(int);

	b3AlignedObjectArray<GLInstanceVertex> vertexArray;
	b3AlignedObjectArray<int> indexArray;

	//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int group = 1;
	int mask = 1;
	int index = 0;
	float scaling[4] = {1, 1, 1, 1};
	int colIndex = 0;

	GLInstanceVertex* cubeVerts = (GLInstanceVertex*)&cube_vertices[0];
	int stride2 = sizeof(GLInstanceVertex);
	b3Assert(stride2 == strideInBytes);

	{
		int childColIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0], strideInBytes, numVertices, scaling);

		b3Vector3 childPositions[3] = {
			b3MakeVector3(0, -2, 0),
			b3MakeVector3(0, 0, 0),
			b3MakeVector3(0, 0, 2)};

		b3AlignedObjectArray<b3GpuChildShape> childShapes;
		int numChildShapes = 3;
		for (int i = 0; i < numChildShapes; i++)
		{
			//for now, only support polyhedral child shapes
			b3GpuChildShape child;
			child.m_shapeIndex = childColIndex;
			b3Vector3 pos = childPositions[i];
			b3Quaternion orn(0, 0, 0, 1);
			for (int v = 0; v < 4; v++)
			{
				child.m_childPosition[v] = pos[v];
				child.m_childOrientation[v] = orn[v];
			}
			childShapes.push_back(child);
			b3Transform tr;
			tr.setIdentity();
			tr.setOrigin(pos);
			tr.setRotation(orn);

			int baseIndex = vertexArray.size();
			for (int j = 0; j < numIndices; j++)
				indexArray.push_back(cube_indices[j] + baseIndex);

			//add transformed graphics vertices and indices
			for (int v = 0; v < numVertices; v++)
			{
				GLInstanceVertex vert = cubeVerts[v];
				b3Vector3 vertPos = b3MakeVector3(vert.xyzw[0], vert.xyzw[1], vert.xyzw[2]);
				b3Vector3 newPos = tr * vertPos;
				vert.xyzw[0] = newPos[0];
				vert.xyzw[1] = newPos[1];
				vert.xyzw[2] = newPos[2];
				vert.xyzw[3] = 0.f;
				vertexArray.push_back(vert);
			}
		}
		colIndex = m_data->m_np->registerCompoundShape(&childShapes);
	}

	//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int shapeId = ci.m_instancingRenderer->registerShape(&vertexArray[0].xyzw[0], vertexArray.size(), &indexArray[0], indexArray.size());

	b3Vector4 colors[4] =
		{
			b3MakeVector4(1, 0, 0, 1),
			b3MakeVector4(0, 1, 0, 1),
			b3MakeVector4(0, 0, 1, 1),
			b3MakeVector4(0, 1, 1, 1),
		};

	int curColor = 0;
	for (int i = 0; i < ci.arraySizeX; i++)
	{
		for (int j = 0; j < ci.arraySizeY; j++)
		{
			for (int k = 0; k < ci.arraySizeZ; k++)

			{
				float mass = 1;  //j==0? 0.f : 1.f;

				b3Vector3 position = b3MakeVector3((-ci.arraySizeX / 2 + i) * ci.gapX, 50 + j * ci.gapY, (-ci.arraySizeZ / 2 + k) * ci.gapZ);
				//b3Quaternion orn(0,0,0,1);
				b3Quaternion orn(b3MakeVector3(1, 0, 0), 0.7);

				b3Vector4 color = colors[curColor];
				curColor++;
				curColor &= 3;
				b3Vector4 scaling = b3MakeVector4(1, 1, 1, 1);
				int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId, position, orn, color, scaling);
				int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass, position, orn, colIndex, index, false);

				index++;
			}
		}
	}
}

void ConcaveSphereScene::setupScene(const ConstructionInfo& ci)
{
	ConcaveScene::setupScene(ci);

	float camPos[4] = {0, 50, 0, 0};  //65.5,4.5,65.5,0};
	//float camPos[4]={1,12.5,1.5,0};
	m_instancingRenderer->setCameraPitch(45);
	m_instancingRenderer->setCameraTargetPosition(camPos);
	m_instancingRenderer->setCameraDistance(40);
}

void ConcaveSphereScene::createDynamicObjects(const ConstructionInfo& ci)
{
	b3Vector4 colors[4] =
		{
			b3MakeVector4(1, 0, 0, 1),
			b3MakeVector4(0, 1, 0, 1),
			b3MakeVector4(0, 1, 1, 1),
			b3MakeVector4(1, 1, 0, 1),
		};

	int index = 0;
	int curColor = 0;
	float radius = 1;
	//int colIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
	int colIndex = m_data->m_np->registerSphereShape(radius);  //>registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
	int prevGraphicsShapeIndex = registerGraphicsSphereShape(ci, radius, false);

	for (int i = 0; i < ci.arraySizeX; i++)
	{
		for (int j = 0; j < ci.arraySizeY; j++)
		{
			for (int k = 0; k < ci.arraySizeZ; k++)
			{
				float mass = 1.f;

				b3Vector3 position = b3MakeVector3(-(ci.arraySizeX / 2) * 8 + i * 8, 50 + j * 8, -(ci.arraySizeZ / 2) * 8 + k * 8);

				//b3Vector3 position(0,-41,0);//0,0,0);//i*radius*3,-41+j*radius*3,k*radius*3);

				b3Quaternion orn(0, 0, 0, 1);

				b3Vector4 color = colors[curColor];
				curColor++;
				curColor &= 3;
				b3Vector4 scaling = b3MakeVector4(radius, radius, radius, 1);
				int id = ci.m_instancingRenderer->registerGraphicsInstance(prevGraphicsShapeIndex, position, orn, color, scaling);
				int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass, position, orn, colIndex, index, false);

				index++;
			}
		}
	}
}*/

CommonExampleInterface* GPUConcaveSceneCreateFunc(struct CommonExampleOptions& options)
{
	return new ConcaveScene(options.m_guiHelper);
}
