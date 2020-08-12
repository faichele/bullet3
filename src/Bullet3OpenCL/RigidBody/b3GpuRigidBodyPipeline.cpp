/*
Copyright (c) 2013 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Erwin Coumans

#include "b3GpuRigidBodyPipeline.h"
#include "b3GpuRigidBodyPipelineInternalData.h"
#include "kernels/integrateKernel.h"
#include "kernels/updateAabbsKernel.h"

#include "kernels/markGhostObjectsKernel.h"
#include "kernels/applyPushPullImpulses.h"

#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "b3GpuNarrowPhase.h"
#include "Bullet3Geometry/b3AabbUtil.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3SapAabb.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuBroadphaseInterface.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
#include "Bullet3Dynamics/ConstraintSolver/b3PgsJacobiSolver.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3UpdateAabbs.h"
#include "Bullet3Collision/BroadPhaseCollision/b3DynamicBvhBroadphase.h"

//#define TEST_OTHER_GPU_SOLVER

#define B3_RIGIDBODY_INTEGRATE_PATH "src/Bullet3OpenCL/RigidBody/kernels/integrateKernel.cl"
#define B3_RIGIDBODY_UPDATEAABB_PATH "src/Bullet3OpenCL/RigidBody/kernels/updateAabbsKernel.cl"
#define B3_RIGIDBODY_MARKGHOSTOBJECTS_PATH "src/Bullet3OpenCL/RigidBody/kernels/markGhostObjectsKernel.cl"
#define B3_RIGIDBODY_APPLYPUSHPULLIMPULSES_PATH "src/Bullet3OpenCL/RigidBody/kernels/applyPushPullImpulses.cl"

bool useBullet2CpuSolver = true;

//choice of contact solver
bool gUseJacobi = true;  //false;
bool gUseDbvt = false;
bool gDumpContactStats = true;
bool gCalcWorldSpaceAabbOnCpu = false;
bool gUseCalculateOverlappingPairsHost = false;
bool gIntegrateOnCpu = false;
bool gClearPairsOnGpu = true;

bool gApplyPushPullBehavioursOnCpu = false;

#define TEST_OTHER_GPU_SOLVER 1
#ifdef TEST_OTHER_GPU_SOLVER
#include "b3GpuJacobiContactSolver.h"
#endif  //TEST_OTHER_GPU_SOLVER

#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyData.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3Contact4.h"
#include "Bullet3OpenCL/RigidBody/b3GpuPgsConstraintSolver.h"

#include "b3GpuPgsContactSolver.h"
#include "b3Solver.h"

#include "Bullet3Collision/NarrowPhaseCollision/b3Config.h"
#include "Bullet3OpenCL/Raycast/b3GpuRaycast.h"

#include "Bullet3Dynamics/shared/b3IntegrateTransforms.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhaseInternalData.h"

b3GpuRigidBodyPipeline::b3GpuRigidBodyPipeline(cl_context ctx, cl_device_id device, cl_command_queue q, class b3GpuNarrowPhase* narrowphase, class b3GpuBroadphaseInterface* broadphaseSap, struct b3DynamicBvhBroadphase* broadphaseDbvt, const b3Config& config)
{
	m_data = new b3GpuRigidBodyPipelineInternalData;
	m_data->m_constraintUid = 0;
	m_data->m_config = config;
	m_data->m_context = ctx;
	m_data->m_device = device;
	m_data->m_queue = q;

	m_data->m_solver = new b3PgsJacobiSolver(true);                            //new b3PgsJacobiSolver(true);
	m_data->m_gpuSolver = new b3GpuPgsConstraintSolver(ctx, device, q, true);  //new b3PgsJacobiSolver(true);

	m_data->m_allAabbsGPU = new b3OpenCLArray<b3SapAabb>(ctx, q, config.m_maxConvexBodies);
	m_data->m_overlappingPairsGPU = new b3OpenCLArray<b3BroadphasePair>(ctx, q, config.m_maxBroadphasePairs);

	// Push-pull behavior data
	m_data->m_collisionFlagsGPU = new b3OpenCLArray<int>(ctx, q, config.m_maxConvexBodies);
	m_data->m_bodiesPushPullBehaviorsGPU = new b3OpenCLArray<b3RigidBodyPushPullBehavior>(ctx, q, config.m_maxConvexBodies);
	m_data->m_bodiesPushPullVelocitiesGPU = new b3OpenCLArray<b3RigidBodyBehaviorVelocities>(ctx, q, config.m_maxConvexBodies);
	m_data->m_pushPullBehaviorApplicationCPU = new b3CpuPushPullBehaviourApplication();

	m_data->m_gpuConstraints = new b3OpenCLArray<b3GpuGenericConstraint>(ctx, q);
#ifdef TEST_OTHER_GPU_SOLVER
	m_data->m_solver3 = new b3GpuJacobiContactSolver(ctx, device, q, config.m_maxBroadphasePairs);
	m_data->m_solver3->setPushPullBehaviorData(m_data->m_bodiesPushPullBehaviorsCPU, m_data->m_bodiesPushPullVelocitiesCPU);
#endif  //	TEST_OTHER_GPU_SOLVER

	m_data->m_solver2 = new b3GpuPgsContactSolver(ctx, device, q, config.m_maxBroadphasePairs);

	m_data->m_raycaster = new b3GpuRaycast(ctx, device, q);

	m_data->m_broadphaseDbvt = broadphaseDbvt;
	m_data->m_broadphaseSap = broadphaseSap;
	m_data->m_narrowphase = narrowphase;
	m_data->m_gravity.setValue(0.f, -9.8f, 0.f);

	cl_int errNum = 0;

	{
		cl_program prog = b3OpenCLUtils::compileCLProgramFromString(m_data->m_context, m_data->m_device, integrateKernelCL, &errNum, "", B3_RIGIDBODY_INTEGRATE_PATH);
		b3Assert(errNum == CL_SUCCESS);
		m_data->m_integrateTransformsKernel = b3OpenCLUtils::compileCLKernelFromString(m_data->m_context, m_data->m_device, integrateKernelCL, "integrateTransformsKernel", &errNum, prog);
		b3Assert(errNum == CL_SUCCESS);
		clReleaseProgram(prog);
	}
	{
		cl_program prog = b3OpenCLUtils::compileCLProgramFromString(m_data->m_context, m_data->m_device, updateAabbsKernelCL, &errNum, "", B3_RIGIDBODY_UPDATEAABB_PATH);
		b3Assert(errNum == CL_SUCCESS);
		m_data->m_updateAabbsKernel = b3OpenCLUtils::compileCLKernelFromString(m_data->m_context, m_data->m_device, updateAabbsKernelCL, "initializeGpuAabbsFull", &errNum, prog);
		b3Assert(errNum == CL_SUCCESS);

		m_data->m_clearOverlappingPairsKernel = b3OpenCLUtils::compileCLKernelFromString(m_data->m_context, m_data->m_device, updateAabbsKernelCL, "clearOverlappingPairsKernel", &errNum, prog);
		b3Assert(errNum == CL_SUCCESS);

		clReleaseProgram(prog);
	}

	{
		cl_program prog = b3OpenCLUtils::compileCLProgramFromString(m_data->m_context, m_data->m_device, markGhostObjectsKernelCL, &errNum, "", B3_RIGIDBODY_MARKGHOSTOBJECTS_PATH);
		b3Assert(errNum == CL_SUCCESS);

		m_data->m_markGhostObjectPairsKernel = b3OpenCLUtils::compileCLKernelFromString(m_data->m_context, m_data->m_device, markGhostObjectsKernelCL, "markGhostObjectPairsKernel", &errNum, prog);
		b3Assert(errNum == CL_SUCCESS);

		clReleaseProgram(prog);
	}

	{
		cl_program prog = b3OpenCLUtils::compileCLProgramFromString(m_data->m_context, m_data->m_device, applyPushPullImpulsesCL, &errNum, "", B3_RIGIDBODY_APPLYPUSHPULLIMPULSES_PATH);
		b3Assert(errNum == CL_SUCCESS);

		m_data->m_applyPushPullImpulsesKernel = b3OpenCLUtils::compileCLKernelFromString(m_data->m_context, m_data->m_device, applyPushPullImpulsesCL, "applyPushPullImpulsesKernel", &errNum, prog);
		b3Assert(errNum == CL_SUCCESS);

		clReleaseProgram(prog);
	}
}

b3GpuRigidBodyPipeline::~b3GpuRigidBodyPipeline()
{
	if (m_data->m_integrateTransformsKernel)
		clReleaseKernel(m_data->m_integrateTransformsKernel);

	if (m_data->m_updateAabbsKernel)
		clReleaseKernel(m_data->m_updateAabbsKernel);

	if (m_data->m_clearOverlappingPairsKernel)
		clReleaseKernel(m_data->m_clearOverlappingPairsKernel);

	if (m_data->m_markGhostObjectPairsKernel)
		clReleaseKernel(m_data->m_markGhostObjectPairsKernel);

	if (m_data->m_applyPushPullImpulsesKernel)
		clReleaseKernel(m_data->m_applyPushPullImpulsesKernel);

	delete m_data->m_raycaster;
	delete m_data->m_solver;
	delete m_data->m_allAabbsGPU;
	delete m_data->m_gpuConstraints;
	delete m_data->m_overlappingPairsGPU;
	delete m_data->m_bodiesPushPullBehaviorsGPU;
	delete m_data->m_bodiesPushPullVelocitiesGPU;
	delete m_data->m_collisionFlagsGPU;

#ifdef TEST_OTHER_GPU_SOLVER
	delete m_data->m_solver3;
#endif  //TEST_OTHER_GPU_SOLVER

	delete m_data->m_solver2;

	delete m_data;
}

void b3GpuRigidBodyPipeline::reset()
{
	m_data->m_gpuConstraints->resize(0);
	m_data->m_cpuConstraints.resize(0);
	m_data->m_allAabbsGPU->resize(0);
	m_data->m_allAabbsCPU.resize(0);

	m_data->m_bodiesPushPullBehaviorsCPU.resize(0);
	m_data->m_bodiesPushPullBehaviorsGPU->resize(0);
	m_data->m_bodiesPushPullVelocitiesCPU.resize(0);
	m_data->m_bodiesPushPullVelocitiesGPU->resize(0);
	
	m_data->m_collisionFlagsGPU->resize(0);
	m_data->m_collisionFlagsCPU.resize(0);
}

void b3GpuRigidBodyPipeline::addConstraint(b3TypedConstraint* constraint)
{
	m_data->m_joints.push_back(constraint);
}

void b3GpuRigidBodyPipeline::removeConstraint(b3TypedConstraint* constraint)
{
	m_data->m_joints.remove(constraint);
}

void b3GpuRigidBodyPipeline::removeConstraintByUid(int uid)
{
	m_data->m_gpuSolver->recomputeBatches();
	//slow linear search
	m_data->m_gpuConstraints->copyToHost(m_data->m_cpuConstraints);
	//remove
	for (int i = 0; i < m_data->m_cpuConstraints.size(); i++)
	{
		if (m_data->m_cpuConstraints[i].m_uid == uid)
		{
			//m_data->m_cpuConstraints.remove(m_data->m_cpuConstraints[i]);
			m_data->m_cpuConstraints.swap(i, m_data->m_cpuConstraints.size() - 1);
			m_data->m_cpuConstraints.pop_back();

			break;
		}
	}

	if (m_data->m_cpuConstraints.size())
	{
		m_data->m_gpuConstraints->copyFromHost(m_data->m_cpuConstraints);
	}
	else
	{
		m_data->m_gpuConstraints->resize(0);
	}
}
int b3GpuRigidBodyPipeline::createPoint2PointConstraint(int bodyA, int bodyB, const float* pivotInA, const float* pivotInB, float breakingThreshold)
{
	m_data->m_gpuSolver->recomputeBatches();
	b3GpuGenericConstraint c;
	c.m_uid = m_data->m_constraintUid;
	m_data->m_constraintUid++;
	c.m_flags = B3_CONSTRAINT_FLAG_ENABLED;
	c.m_rbA = bodyA;
	c.m_rbB = bodyB;
	c.m_pivotInA.setValue(pivotInA[0], pivotInA[1], pivotInA[2]);
	c.m_pivotInB.setValue(pivotInB[0], pivotInB[1], pivotInB[2]);
	c.m_breakingImpulseThreshold = breakingThreshold;
	c.m_constraintType = B3_GPU_POINT2POINT_CONSTRAINT_TYPE;
	m_data->m_cpuConstraints.push_back(c);
	return c.m_uid;
}
int b3GpuRigidBodyPipeline::createFixedConstraint(int bodyA, int bodyB, const float* pivotInA, const float* pivotInB, const float* relTargetAB, float breakingThreshold)
{
	m_data->m_gpuSolver->recomputeBatches();
	b3GpuGenericConstraint c;
	c.m_uid = m_data->m_constraintUid;
	m_data->m_constraintUid++;
	c.m_flags = B3_CONSTRAINT_FLAG_ENABLED;
	c.m_rbA = bodyA;
	c.m_rbB = bodyB;
	c.m_pivotInA.setValue(pivotInA[0], pivotInA[1], pivotInA[2]);
	c.m_pivotInB.setValue(pivotInB[0], pivotInB[1], pivotInB[2]);
	c.m_relTargetAB.setValue(relTargetAB[0], relTargetAB[1], relTargetAB[2], relTargetAB[3]);
	c.m_breakingImpulseThreshold = breakingThreshold;
	c.m_constraintType = B3_GPU_FIXED_CONSTRAINT_TYPE;

	m_data->m_cpuConstraints.push_back(c);
	return c.m_uid;
}

void b3GpuRigidBodyPipeline::stepSimulation(float deltaTime)
{
	//update worldspace AABBs from local AABB/worldtransform
	{
		B3_PROFILE("setupGpuAabbs");
		setupGpuAabbsFull();
	}

	int numPairs = 0;

	//compute overlapping pairs
	{
		if (gUseDbvt)
		{
			{
				B3_PROFILE("setAabb");
				m_data->m_allAabbsGPU->copyToHost(m_data->m_allAabbsCPU);
				for (int i = 0; i < m_data->m_allAabbsCPU.size(); i++)
				{
					b3Vector3 aabbMin = b3MakeVector3(m_data->m_allAabbsCPU[i].m_min[0], m_data->m_allAabbsCPU[i].m_min[1], m_data->m_allAabbsCPU[i].m_min[2]);
					b3Vector3 aabbMax = b3MakeVector3(m_data->m_allAabbsCPU[i].m_max[0], m_data->m_allAabbsCPU[i].m_max[1], m_data->m_allAabbsCPU[i].m_max[2]);
					m_data->m_broadphaseDbvt->setAabb(i, aabbMin, aabbMax, 0);
				}
			}

			{
				B3_PROFILE("calculateOverlappingPairs");
				m_data->m_broadphaseDbvt->calculateOverlappingPairs();
			}
			numPairs = m_data->m_broadphaseDbvt->getOverlappingPairCache()->getNumOverlappingPairs();
		}
		else
		{
			if (gUseCalculateOverlappingPairsHost)
			{
				m_data->m_broadphaseSap->calculateOverlappingPairsHost(m_data->m_config.m_maxBroadphasePairs);
			}
			else
			{
				m_data->m_broadphaseSap->calculateOverlappingPairs(m_data->m_config.m_maxBroadphasePairs);
			}
			numPairs = m_data->m_broadphaseSap->getNumOverlap();
		}
	}

	//compute contact points
	printf("numPairs = %d\n", numPairs);

	int numContacts = 0;

	int numBodies = m_data->m_narrowphase->getNumRigidBodies();

	if (numPairs)
	{
		cl_mem pairs = 0;
		cl_mem aabbsWS = 0;
		cl_mem collisionFlags = 0;

		if (gUseDbvt)
		{
			B3_PROFILE("m_overlappingPairsGPU->copyFromHost");
			m_data->m_overlappingPairsGPU->copyFromHost(m_data->m_broadphaseDbvt->getOverlappingPairCache()->getOverlappingPairArray());
			pairs = m_data->m_overlappingPairsGPU->getBufferCL();
			aabbsWS = m_data->m_allAabbsGPU->getBufferCL();
		}
		else
		{
			pairs = m_data->m_broadphaseSap->getOverlappingPairBuffer();
			aabbsWS = m_data->m_broadphaseSap->getAabbBufferWS();
		}

		m_data->m_collisionFlagsGPU->resize(numBodies, false);

		m_data->m_overlappingPairsGPU->resize(numPairs);

		//mark the contacts for each pair as 'unused'
		if (numPairs)
		{
			b3OpenCLArray<b3BroadphasePair> gpuPairs(this->m_data->m_context, m_data->m_queue);
			gpuPairs.setFromOpenCLBuffer(pairs, numPairs);

			if (gClearPairsOnGpu)
			{
				//b3AlignedObjectArray<b3BroadphasePair> hostPairs;//just for debugging
				//gpuPairs.copyToHost(hostPairs);

				b3LauncherCL launcher(m_data->m_queue, m_data->m_clearOverlappingPairsKernel, "clearOverlappingPairsKernel");
				launcher.setBuffer(pairs);
				launcher.setConst(numPairs);
				launcher.launch1D(numPairs);

				//gpuPairs.copyToHost(hostPairs);
			}
			else
			{
				b3AlignedObjectArray<b3BroadphasePair> hostPairs;
				gpuPairs.copyToHost(hostPairs);

				for (int i = 0; i < hostPairs.size(); i++)
				{
					hostPairs[i].z = 0xffffffff;
				}

				gpuPairs.copyFromHost(hostPairs);
			}
			//b3OpenCLArray<b3BroadphasePair> gpuCollisionFlags(this->m_data->m_context, m_data->m_queue);
			//gpuCollisionFlags.setFromOpenCLBuffer(collisionFlags, numBodies);

			/*b3AlignedObjectArray<int>& collisionFilters = m_data->m_broadphaseSap->getCollisionFilterGroups();
			for (size_t k = 0; k < collisionFilters.size(); ++k)
				std::cout << "collisionFilters[" << k << "] = " << collisionFilters[k] << std::endl;*/

			collisionFlags = m_data->m_collisionFlagsGPU->getBufferCL();

			b3LauncherCL launcher(m_data->m_queue, m_data->m_markGhostObjectPairsKernel, "markGhostObjectPairsKernel");

			//std::cout << "setBuffer(pairs)" << std::endl;
			launcher.setBuffer(pairs);
			//std::cout << "setBuffer(collisionFlags)" << std::endl;
			launcher.setBuffer(collisionFlags);

			//std::cout << "setConst(numPairs)" << std::endl;
			launcher.setConst(numPairs);
			//std::cout << "setConst(numBodies)" << std::endl;
			launcher.setConst(numBodies);

			launcher.launch1D(numPairs);

			/*b3AlignedObjectArray<b3BroadphasePair> hostPairs;
			gpuPairs.copyToHost(hostPairs);

			std::cout << "Broadphase pair w components GPU dump: " << hostPairs.size() << std::endl;
			for (int k = 0; k < hostPairs.size(); k++)
				std::cout << "Pair " << k << ": " << hostPairs[k].w << std::endl;

			for (int i = 0; i < hostPairs.size(); i++)
			{
				if (hostPairs[i].x < collisionFilters.size())
				{
					if (collisionFilters[hostPairs[i].x] & CF_GHOST_OBJECT)
						hostPairs[i].w = CF_GHOST_OBJECT;
				}

				if (hostPairs[i].y < collisionFilters.size())
				{
					if (collisionFilters[hostPairs[i].y] & CF_GHOST_OBJECT)
						hostPairs[i].w = CF_GHOST_OBJECT;
				}
			}

			std::cout << "Broadphase pair w components CPU dump: " << hostPairs.size() << std::endl;
			for (int k = 0; k < hostPairs.size(); k++)
				std::cout << "Pair " << k << ": " << hostPairs[k].w << std::endl;*/

			// Store overlapping pairs for usage with push-pull behaviors in integration
			gpuPairs.copyToHost(m_data->m_overlappingPairsCPU);
		}

		m_data->m_narrowphase->computeContacts(pairs, numPairs, aabbsWS, numBodies);
		numContacts = m_data->m_narrowphase->getNumContactsGpu();

		if (gUseDbvt)
		{
			///store the cached information (contact locations in the 'z' component)
			B3_PROFILE("m_overlappingPairsGPU->copyToHost");
			m_data->m_overlappingPairsGPU->copyToHost(m_data->m_broadphaseDbvt->getOverlappingPairCache()->getOverlappingPairArray());
		}
	}

	if (numContacts)
	{
		if (gDumpContactStats)
		{
			const b3Contact4* contacts = m_data->m_narrowphase->getContactsCPU();
			printf("numContacts = %d\n", numContacts);

			int totalPoints = 0;
			for (int i = 0; i < numContacts; i++)
			{
				totalPoints += contacts->getNPoints();
			}
			printf("totalPoints = %d\n", totalPoints);
		}
	}

	//Filter out contact points involving ghost objects
	const b3AlignedObjectArray<int>& collisionFlagsCPU = m_data->m_collisionFlagsCPU;
	b3AlignedObjectArray<b3Contact4> filteredContacts;
	if (numContacts)
	{
		const b3Contact4* contacts = m_data->m_narrowphase->getContactsCPU();
		for (int k = 0; k < numContacts; ++k)
		{
			int bodyIdxA = contacts[k].getBodyA();
			int bodyIdxB = contacts[k].getBodyB();
			// printf("Checking contact pair %i (%i - %i) if one body is-a ghost object.\n", k, bodyIdxA, bodyIdxB);
			bool ghostContact = false;
			if (bodyIdxA < collisionFlagsCPU.size() && bodyIdxB < collisionFlagsCPU.size())
			{
				if (collisionFlagsCPU[bodyIdxA] & b3CollisionFlags::CF_GHOST_OBJECT)
				{
					ghostContact = true;
					printf("Body A (%i) is a ghost object\n", bodyIdxA);
				}

				if (collisionFlagsCPU[bodyIdxB] & b3CollisionFlags::CF_GHOST_OBJECT)
				{
					ghostContact = true;
					printf("Body B (%i) is a ghost object\n", bodyIdxB);
				}

				if (!ghostContact)
					filteredContacts.push_back(contacts[k]);
			}
		}

		printf("contacts count = %i vs. filteredContacts count = %i\n", numContacts, filteredContacts.size());
	}

	//convert contact points to contact constraints

	//solve constraints

	b3OpenCLArray<b3RigidBodyData> gpuBodies(m_data->m_context, m_data->m_queue, 0, true);
	gpuBodies.setFromOpenCLBuffer(m_data->m_narrowphase->getBodiesGpu(), m_data->m_narrowphase->getNumRigidBodies());
	b3OpenCLArray<b3InertiaData> gpuInertias(m_data->m_context, m_data->m_queue, 0, true);
	gpuInertias.setFromOpenCLBuffer(m_data->m_narrowphase->getBodyInertiasGpu(), m_data->m_narrowphase->getNumRigidBodies());

	b3OpenCLArray<b3Contact4> gpuContacts(m_data->m_context, m_data->m_queue, 0, true);
	// gpuContacts.setFromOpenCLBuffer(m_data->m_narrowphase->getContactsGpu(), m_data->m_narrowphase->getNumContactsGpu());
	gpuContacts.copyFromHost(filteredContacts, true);

	int numJoints = m_data->m_joints.size() ? m_data->m_joints.size() : m_data->m_cpuConstraints.size();
	if (useBullet2CpuSolver && numJoints)
	{
		//	b3AlignedObjectArray<b3Contact4> hostContacts;
		//gpuContacts.copyToHost(hostContacts);
		{
			bool useGpu = m_data->m_joints.size() == 0;

			//			b3Contact4* contacts = numContacts? &hostContacts[0]: 0;
			//m_data->m_solver->solveContacts(m_data->m_narrowphase->getNumBodiesGpu(),&hostBodies[0],&hostInertias[0],numContacts,contacts,numJoints, joints);
			if (useGpu)
			{
				std::cout << "m_data->m_gpuSolver->solveJoints()" << std::endl;
				m_data->m_gpuSolver->solveJoints(m_data->m_narrowphase->getNumRigidBodies(), &gpuBodies, &gpuInertias, numJoints, m_data->m_gpuConstraints);
			}
			else
			{
				b3AlignedObjectArray<b3RigidBodyData> hostBodies;
				gpuBodies.copyToHost(hostBodies);
				b3AlignedObjectArray<b3InertiaData> hostInertias;
				gpuInertias.copyToHost(hostInertias);

				b3TypedConstraint** joints = numJoints ? &m_data->m_joints[0] : 0;

				std::cout << "m_data->m_solver->solveContacts()" << std::endl;
				m_data->m_solver->solveContacts(m_data->m_narrowphase->getNumRigidBodies(), &hostBodies[0], &hostInertias[0], 0, 0, numJoints, joints);
				gpuBodies.copyFromHost(hostBodies);
			}
		}
	}

	if (filteredContacts.size() > 0)
	{
#ifdef TEST_OTHER_GPU_SOLVER

		if (gUseJacobi)
		{
			bool useGpu = true;
			if (useGpu)
			{
				bool forceHost = false;
				if (forceHost)
				{
					b3AlignedObjectArray<b3RigidBodyData> hostBodies;
					b3AlignedObjectArray<b3InertiaData> hostInertias;
					b3AlignedObjectArray<b3Contact4> hostContacts;

					{
						B3_PROFILE("copyToHost");
						gpuBodies.copyToHost(hostBodies);
						gpuInertias.copyToHost(hostInertias);

						// gpuContacts.copyToHost(hostContacts);
						hostContacts = filteredContacts;
					}

					{
						b3JacobiSolverInfo solverInfo;
						m_data->m_solver3->solveGroupHost(&hostBodies[0], &hostInertias[0], hostBodies.size(), &hostContacts[0], hostContacts.size(), solverInfo);
					}
					{
						B3_PROFILE("copyFromHost");
						gpuBodies.copyFromHost(hostBodies);
					}
				}
				else
				{
					int static0Index = m_data->m_narrowphase->getStatic0Index();
					b3JacobiSolverInfo solverInfo;
					//m_data->m_solver3->solveContacts(    >solveGroup(&gpuBodies, &gpuInertias, &gpuContacts,solverInfo);
					//m_data->m_solver3->solveContacts(m_data->m_narrowphase->getNumBodiesGpu(),&hostBodies[0],&hostInertias[0],numContacts,&hostContacts[0]);
					
					std::cout << "m_data->m_solver3->solveContacts()" << std::endl;
					m_data->m_solver3->solveContacts(numBodies, gpuBodies.getBufferCL(), gpuInertias.getBufferCL(), filteredContacts.size() /*numContacts*/, 
						gpuContacts.getBufferCL(), m_data->m_config, static0Index, 
						m_data->m_bodiesPushPullBehaviorsCPU, m_data->m_bodiesPushPullVelocitiesCPU
						);
				}
			}
			else
			{
				b3AlignedObjectArray<b3RigidBodyData> hostBodies;
				gpuBodies.copyToHost(hostBodies);
				b3AlignedObjectArray<b3InertiaData> hostInertias;
				gpuInertias.copyToHost(hostInertias);
				b3AlignedObjectArray<b3Contact4> hostContacts;
				gpuContacts.copyToHost(hostContacts);
				{
					//m_data->m_solver->solveContacts(m_data->m_narrowphase->getNumBodiesGpu(),&hostBodies[0],&hostInertias[0],numContacts,&hostContacts[0]);
				}
				gpuBodies.copyFromHost(hostBodies);
			}
		}
		else
#endif  //TEST_OTHER_GPU_SOLVER
		{
			int static0Index = m_data->m_narrowphase->getStatic0Index();
			std::cout << "m_data->m_solver2->solveContacts()" << std::endl;
			m_data->m_solver2->solveContacts(numBodies, gpuBodies.getBufferCL(), gpuInertias.getBufferCL(), filteredContacts.size() /*numContacts*/, gpuContacts.getBufferCL(), m_data->m_config, static0Index);

			//m_data->m_solver4->solveContacts(m_data->m_narrowphase->getNumBodiesGpu(), gpuBodies.getBufferCL(), gpuInertias.getBufferCL(), numContacts, gpuContacts.getBufferCL());

			/*m_data->m_solver3->solveContactConstraintHost(
			(b3OpenCLArray<RigidBodyBase::Body>*)&gpuBodies,
			(b3OpenCLArray<RigidBodyBase::Inertia>*)&gpuInertias,
			(b3OpenCLArray<Constraint4>*) &gpuContacts,
			0,numContacts,256);
			*/
		}
	}

	integrate(deltaTime);
}

void b3GpuRigidBodyPipeline::integrate(float timeStep)
{
	//integrate
	int numBodies = m_data->m_narrowphase->getNumRigidBodies();
	float angularDamp = 0.99f;

	if (gIntegrateOnCpu)
	{
		if (numBodies)
		{
			b3GpuNarrowPhaseInternalData* npData = m_data->m_narrowphase->getInternalData();
			npData->m_bodyBufferGPU->copyToHost(*npData->m_bodyBufferCPU);

			b3RigidBodyData_t* bodies = &npData->m_bodyBufferCPU->at(0);

			for (int nodeID = 0; nodeID < numBodies; nodeID++)
			{
				integrateSingleTransform(bodies, nodeID, timeStep, angularDamp, m_data->m_gravity);
			}
			npData->m_bodyBufferGPU->copyFromHost(*npData->m_bodyBufferCPU);
		}
	}
	else
	{
		if (numBodies)
		{
			if (m_data->m_bodiesPushPullBehaviorsCPU.size() > 0)
			{
				if (gApplyPushPullBehavioursOnCpu)
				{
					b3GpuNarrowPhaseInternalData* npData = m_data->m_narrowphase->getInternalData();
					npData->m_bodyBufferGPU->copyToHost(*npData->m_bodyBufferCPU);

					b3RigidBodyData_t* bodies = &npData->m_bodyBufferCPU->at(0);

					m_data->m_pushPullBehaviorApplicationCPU->applyPushPullBehaviours(
						bodies,
						numBodies,
						timeStep,
						angularDamp,
						m_data->m_gravity,
						m_data->m_overlappingPairsCPU,
						m_data->m_bodiesPushPullBehaviorsCPU,
						m_data->m_bodiesPushPullVelocitiesCPU);

					npData->m_bodyBufferGPU->copyFromHost(*npData->m_bodyBufferCPU);
				}
				else
				{
					b3LauncherCL launcher(m_data->m_queue, m_data->m_applyPushPullImpulsesKernel, "m_applyPushPullImpulsesKernel");
					launcher.setBuffer(m_data->m_narrowphase->getBodiesGpu());

					launcher.setConst(numBodies);
					launcher.setConst(timeStep);
					launcher.setConst(angularDamp);
					launcher.setConst(m_data->m_gravity);

					launcher.setBuffer(m_data->m_overlappingPairsGPU->getBufferCL());
					launcher.setBuffer(m_data->m_bodiesPushPullBehaviorsGPU->getBufferCL());
					launcher.setBuffer(m_data->m_bodiesPushPullVelocitiesGPU->getBufferCL());
					launcher.setConst(m_data->m_overlappingPairsCPU.size());
					launcher.setConst(m_data->m_bodiesPushPullBehaviorsCPU.size());

					launcher.launch1D(numBodies);

					/*b3AlignedObjectArray<b3RigidBodyBehaviorVelocities> hostPushPullVelocities;  //just for debugging
					m_data->m_bodiesPushPullVelocitiesGPU->copyToHost(hostPushPullVelocities);

					std::cout << "=== Push-pull velocity entries: " << hostPushPullVelocities.size() << " ===" << std::endl;
					for (int k = 0; k < hostPushPullVelocities.size(); ++k)
					{
						std::cout << "Body ID " << hostPushPullVelocities[k].m_bodyID << ": linear v = (" << hostPushPullVelocities[k].m_linearAcc.x << ", " << hostPushPullVelocities[k].m_linearAcc.y << ", " << hostPushPullVelocities[k].m_linearAcc.z << ")"
									<< "angular v = (" << hostPushPullVelocities[k].m_angularAcc.x << ", " << hostPushPullVelocities[k].m_angularAcc.y << ", " << hostPushPullVelocities[k].m_angularAcc.z << ") "
									<< std::endl;
					}
					std::cout << "===========================================================================" << std::endl;*/
				}
			}
		}
		else
		{
			b3LauncherCL launcher(m_data->m_queue, m_data->m_integrateTransformsKernel, "m_integrateTransformsKernel");
			launcher.setBuffer(m_data->m_narrowphase->getBodiesGpu());

			launcher.setConst(numBodies);
			launcher.setConst(timeStep);
			launcher.setConst(angularDamp);
			launcher.setConst(m_data->m_gravity);
			launcher.launch1D(numBodies);
		}
	}
}

void b3GpuRigidBodyPipeline::setupGpuAabbsFull()
{
	cl_int ciErrNum = 0;

	int numBodies = m_data->m_narrowphase->getNumRigidBodies();
	if (!numBodies)
		return;

	if (gCalcWorldSpaceAabbOnCpu)
	{
		if (numBodies)
		{
			if (gUseDbvt)
			{
				m_data->m_allAabbsCPU.resize(numBodies);
				m_data->m_narrowphase->readbackAllBodiesToCpu();
				for (int i = 0; i < numBodies; i++)
				{
					b3ComputeWorldAabb(i, m_data->m_narrowphase->getBodiesCpu(), m_data->m_narrowphase->getCollidablesCpu(), m_data->m_narrowphase->getLocalSpaceAabbsCpu(), &m_data->m_allAabbsCPU[0]);
				}
				m_data->m_allAabbsGPU->copyFromHost(m_data->m_allAabbsCPU);
			}
			else
			{
				m_data->m_broadphaseSap->getAllAabbsCPU().resize(numBodies);
				m_data->m_narrowphase->readbackAllBodiesToCpu();
				for (int i = 0; i < numBodies; i++)
				{
					b3ComputeWorldAabb(i, m_data->m_narrowphase->getBodiesCpu(), m_data->m_narrowphase->getCollidablesCpu(), m_data->m_narrowphase->getLocalSpaceAabbsCpu(), &m_data->m_broadphaseSap->getAllAabbsCPU()[0]);
				}
				m_data->m_broadphaseSap->getAllAabbsGPU().copyFromHost(m_data->m_broadphaseSap->getAllAabbsCPU());
				//m_data->m_broadphaseSap->writeAabbsToGpu();
			}
		}
	}
	else
	{
		//__kernel void initializeGpuAabbsFull(  const int numNodes, __global Body* gBodies,__global Collidable* collidables, __global b3AABBCL* plocalShapeAABB, __global b3AABBCL* pAABB)
		b3LauncherCL launcher(m_data->m_queue, m_data->m_updateAabbsKernel, "m_updateAabbsKernel");
		launcher.setConst(numBodies);
		cl_mem bodies = m_data->m_narrowphase->getBodiesGpu();
		launcher.setBuffer(bodies);
		cl_mem collidables = m_data->m_narrowphase->getCollidablesGpu();
		launcher.setBuffer(collidables);
		cl_mem localAabbs = m_data->m_narrowphase->getAabbLocalSpaceBufferGpu();
		launcher.setBuffer(localAabbs);

		cl_mem worldAabbs = 0;
		if (gUseDbvt)
		{
			worldAabbs = m_data->m_allAabbsGPU->getBufferCL();
		}
		else
		{
			worldAabbs = m_data->m_broadphaseSap->getAabbBufferWS();
		}
		launcher.setBuffer(worldAabbs);
		launcher.launch1D(numBodies);

		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}

	/*
	b3AlignedObjectArray<b3SapAabb> aabbs;
	m_data->m_broadphaseSap->m_allAabbsGPU.copyToHost(aabbs);

	printf("numAabbs = %d\n",  aabbs.size());

	for (int i=0;i<aabbs.size();i++)
	{
		printf("aabb[%d].m_min=%f,%f,%f,%d\n",i,aabbs[i].m_minVec[0],aabbs[i].m_minVec[1],aabbs[i].m_minVec[2],aabbs[i].m_minIndices[3]);
		printf("aabb[%d].m_max=%f,%f,%f,%d\n",i,aabbs[i].m_maxVec[0],aabbs[i].m_maxVec[1],aabbs[i].m_maxVec[2],aabbs[i].m_signedMaxIndices[3]);

	};
	*/
}

cl_mem b3GpuRigidBodyPipeline::getBodyBuffer()
{
	return m_data->m_narrowphase->getBodiesGpu();
}

int b3GpuRigidBodyPipeline::getNumBodies() const
{
	return m_data->m_narrowphase->getNumRigidBodies();
}

void b3GpuRigidBodyPipeline::setGravity(const float* grav)
{
	m_data->m_gravity.setValue(grav[0], grav[1], grav[2]);
}

void b3GpuRigidBodyPipeline::copyConstraintsToHost()
{
	m_data->m_gpuConstraints->copyToHost(m_data->m_cpuConstraints);
}

void b3GpuRigidBodyPipeline::writeAllInstancesToGpu()
{
	m_data->m_allAabbsGPU->copyFromHost(m_data->m_allAabbsCPU);
	m_data->m_gpuConstraints->copyFromHost(m_data->m_cpuConstraints);
	m_data->m_collisionFlagsCPU.resize(m_data->m_broadphaseSap->getCollisionFilterMasks().size());

	// TODO: Make this work with rigid bodies being added and removed at runtime!
	b3AlignedObjectArray<int> collisionMasks = m_data->m_broadphaseSap->getCollisionFilterGroups();
	for (size_t k = 0; k < collisionMasks.size(); ++k)
		m_data->m_collisionFlagsCPU[k] = collisionMasks[k];

	m_data->m_collisionFlagsGPU->copyFromHost(m_data->m_collisionFlagsCPU);

	m_data->m_bodiesPushPullBehaviorsGPU->copyFromHost(m_data->m_bodiesPushPullBehaviorsCPU);
}

int b3GpuRigidBodyPipeline::registerPhysicsInstance(float mass, const float* position, const float* orientation, int collidableIndex, int userIndex, bool writeInstanceToGpu, int collisionFlags, int collisionGroupMask)
{
	b3Vector3 aabbMin = b3MakeVector3(0, 0, 0), aabbMax = b3MakeVector3(0, 0, 0);

	if (collidableIndex >= 0)
	{
		b3SapAabb localAabb = m_data->m_narrowphase->getLocalSpaceAabb(collidableIndex);
		b3Vector3 localAabbMin = b3MakeVector3(localAabb.m_min[0], localAabb.m_min[1], localAabb.m_min[2]);
		b3Vector3 localAabbMax = b3MakeVector3(localAabb.m_max[0], localAabb.m_max[1], localAabb.m_max[2]);

		b3Scalar margin = 0.01f;
		b3Transform t;
		t.setIdentity();
		t.setOrigin(b3MakeVector3(position[0], position[1], position[2]));
		t.setRotation(b3Quaternion(orientation[0], orientation[1], orientation[2], orientation[3]));
		b3TransformAabb(localAabbMin, localAabbMax, margin, t, aabbMin, aabbMax);
	}
	else
	{
		b3Error("registerPhysicsInstance using invalid collidableIndex\n");
		return -1;
	}

	bool writeToGpu = false;
	int bodyIndex = m_data->m_narrowphase->getNumRigidBodies();
	bodyIndex = m_data->m_narrowphase->registerRigidBody(collidableIndex, mass, position, orientation, &aabbMin.getX(), &aabbMax.getX(), writeToGpu);

	if (bodyIndex >= 0)
	{
		if (gUseDbvt)
		{
			m_data->m_broadphaseDbvt->createProxy(aabbMin, aabbMax, bodyIndex, 0, collisionFlags, collisionGroupMask);
			b3SapAabb aabb;
			for (int i = 0; i < 3; i++)
			{
				aabb.m_min[i] = aabbMin[i];
				aabb.m_max[i] = aabbMax[i];
				aabb.m_minIndices[3] = bodyIndex;
			}
			m_data->m_allAabbsCPU.push_back(aabb);
			if (writeInstanceToGpu)
			{
				m_data->m_allAabbsGPU->copyFromHost(m_data->m_allAabbsCPU);
			}
		}
		else
		{
			if (mass)
			{
				m_data->m_broadphaseSap->createProxy(aabbMin, aabbMax, bodyIndex, collisionFlags, collisionGroupMask);  //m_dispatcher);
			}
			else
			{
				m_data->m_broadphaseSap->createLargeProxy(aabbMin, aabbMax, bodyIndex, collisionFlags, collisionGroupMask);  //m_dispatcher);
			}
		}
	}

	/*
	if (mass>0.f)
		m_numDynamicPhysicsInstances++;

	m_numPhysicsInstances++;
	*/

	return bodyIndex;
}

void b3GpuRigidBodyPipeline::setPhysicsInstancePushPullBehavior(int instanceIndex, float* translationalVelocity, float* rotationalVelocity, float* anchorPoint, float* anchorOrientation)
{
	bool behaviorExists = false;
	int behaviorIndex = -1;
	for (int k = 0; k < m_data->m_bodiesPushPullBehaviorsCPU.size(); ++k)
	{
		if (m_data->m_bodiesPushPullBehaviorsCPU[k].m_bodyID == instanceIndex)
		{
			behaviorExists = true;
			behaviorIndex = k;
			break;
		}
	}

	if (!behaviorExists)
	{
		b3RigidBodyPushPullBehavior ppBehavior;
		ppBehavior.m_bodyID = instanceIndex;
		ppBehavior.m_linearAcc = b3MakeFloat4(translationalVelocity[0], translationalVelocity[1], translationalVelocity[2]);
		ppBehavior.m_angularAcc = b3MakeFloat4(rotationalVelocity[0], rotationalVelocity[1], rotationalVelocity[2]);

		ppBehavior.m_bodyPosition = b3MakeFloat4(anchorPoint[0], anchorPoint[1], anchorPoint[2]);
		ppBehavior.m_bodyOrientation = b3Quaternion(anchorOrientation[0], anchorOrientation[1], anchorOrientation[2], anchorOrientation[3]);
		m_data->m_bodiesPushPullBehaviorsCPU.push_back(ppBehavior);

		m_data->m_pushPullBehaviorApplicationCPU->registerPushPullBehavior(ppBehavior);
	}
	else
	{
		m_data->m_bodiesPushPullBehaviorsCPU[behaviorIndex].m_linearAcc = b3MakeFloat4(translationalVelocity[0], translationalVelocity[1], translationalVelocity[2]);
		m_data->m_bodiesPushPullBehaviorsCPU[behaviorIndex].m_angularAcc = b3MakeFloat4(rotationalVelocity[0], rotationalVelocity[1], rotationalVelocity[2]);
		m_data->m_bodiesPushPullBehaviorsCPU[behaviorIndex].m_bodyPosition = b3MakeFloat4(anchorPoint[0], anchorPoint[1], anchorPoint[2]);
		m_data->m_bodiesPushPullBehaviorsCPU[behaviorIndex].m_bodyOrientation = b3Quaternion(anchorOrientation[0], anchorOrientation[1], anchorOrientation[2], anchorOrientation[3]);
	
		m_data->m_pushPullBehaviorApplicationCPU->unregisterPushPullBehavior(m_data->m_bodiesPushPullBehaviorsCPU[behaviorIndex]);
		m_data->m_pushPullBehaviorApplicationCPU->registerPushPullBehavior(m_data->m_bodiesPushPullBehaviorsCPU[behaviorIndex]);
	}

	if (m_data->m_solver3)
		m_data->m_solver3->setPushPullBehaviorData(m_data->m_bodiesPushPullBehaviorsCPU, m_data->m_bodiesPushPullVelocitiesCPU);
}

void b3GpuRigidBodyPipeline::removePhysicsInstancePushPullBehavior(int instanceIndex)
{
	bool behaviorExists = false;
	int behaviorIndex = -1;
	for (int k = 0; k < m_data->m_bodiesPushPullBehaviorsCPU.size(); ++k)
	{
		if (m_data->m_bodiesPushPullBehaviorsCPU[k].m_bodyID == instanceIndex)
		{
			behaviorExists = true;
			behaviorIndex = k;
			break;
		}
	}

	if (behaviorExists)
	{
		m_data->m_bodiesPushPullBehaviorsCPU.removeAtIndex(behaviorIndex);
	}

	if (m_data->m_solver3)
		m_data->m_solver3->setPushPullBehaviorData(m_data->m_bodiesPushPullBehaviorsCPU, m_data->m_bodiesPushPullVelocitiesCPU);
}

const b3AlignedObjectArray<b3RigidBodyPushPullBehavior>& b3GpuRigidBodyPipeline::getPushPullBehaviors() const
{
	return m_data->m_bodiesPushPullBehaviorsCPU;
}

void b3GpuRigidBodyPipeline::castRays(const b3AlignedObjectArray<b3RayInfo>& rays, b3AlignedObjectArray<b3RayHit>& hitResults)
{
	this->m_data->m_raycaster->castRays(rays, hitResults,
										getNumBodies(), this->m_data->m_narrowphase->getBodiesCpu(),
										m_data->m_narrowphase->getNumCollidablesGpu(), m_data->m_narrowphase->getCollidablesCpu(),
										m_data->m_narrowphase->getInternalData(), m_data->m_broadphaseSap);
}

const b3Contact4* b3GpuRigidBodyPipeline::getContacts()
{
	if (m_data->m_narrowphase)
		return m_data->m_narrowphase->getContactsCPU();

	return nullptr;
}

const unsigned int b3GpuRigidBodyPipeline::getNumContacts()
{
	if (m_data->m_narrowphase)
		return m_data->m_narrowphase->getNumContactsGpu();

	return 0;
}

const b3AlignedObjectArray<int>& b3GpuRigidBodyPipeline::getCollisionFlags()
{
	return m_data->m_collisionFlagsCPU;
}
