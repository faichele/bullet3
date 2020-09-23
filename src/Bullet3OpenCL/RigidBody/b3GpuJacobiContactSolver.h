
#ifndef B3_GPU_JACOBI_CONTACT_SOLVER_H
#define B3_GPU_JACOBI_CONTACT_SOLVER_H
#include "Bullet3OpenCL/Initialize/b3OpenCLInclude.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyData.h"

#include "Bullet3Common/shared/b3Int2.h"
#include "Bullet3OpenCL/RigidBody/b3GpuConstraint4.h"

#include "Bullet3Collision/NarrowPhaseCollision/shared/b3Contact4Data.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3OpenCLArray.h"

#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyBehavior.h"

#include "Utils/b3Clock.h"

#include <vector>
#include <map>

class b3TypedConstraint;

struct b3JacobiSolverInfo
{
	int m_fixedBodyIndex;

	float m_deltaTime;
	float m_positionDrift;
	float m_positionConstraintCoeff;
	int m_numIterations;

	b3JacobiSolverInfo()
		: m_fixedBodyIndex(0),
		  m_deltaTime(1. / 60.f),
		  m_positionDrift(0.005f),
		  m_positionConstraintCoeff(0.99f),
		  m_numIterations(7)
	{
	}
};
class b3GpuJacobiContactSolver
{
protected:
	struct b3GpuJacobiSolverInternalData* m_data;

	cl_context m_context;
	cl_device_id m_device;
	cl_command_queue m_queue;

	b3Clock m_clock;

public:
	b3GpuJacobiContactSolver(cl_context ctx, cl_device_id device, cl_command_queue queue, int pairCapacity);
	virtual ~b3GpuJacobiContactSolver();

	void setPushPullBehaviorData(const b3AlignedObjectArray<b3RigidBodyPushPullBehavior>&, const b3AlignedObjectArray<b3RigidBodyBehaviorVelocities>&);

	void solveContacts(int numBodies, cl_mem bodyBuf, cl_mem inertiaBuf, int numContacts, cl_mem contactBuf, const struct b3Config& config, int static0Index,
					   cl_mem pushPullVelocities);

					   //b3AlignedObjectArray<b3RigidBodyPushPullBehavior>& pushPullBehaviours, b3AlignedObjectArray<b3RigidBodyBehaviorVelocities>& pushPullVelocities);
	void solveGroupHost(b3RigidBodyData* bodies, b3InertiaData* inertias, int numBodies, struct b3Contact4* manifoldPtr, int numManifolds, const b3JacobiSolverInfo& solverInfo, 
		b3AlignedObjectArray<b3RigidBodyPushPullBehavior>& pushPullBehaviors, b3AlignedObjectArray<b3RigidBodyBehaviorVelocities>& pushPullVelocities, const std::map<int, std::vector<int>>& ppMap);

	b3AlignedObjectArray<b3Int2>& getContactConstraintOffsetsCPU();
	b3AlignedObjectArray<b3GpuConstraint4>& getContactConstraintsCPU();
	b3AlignedObjectArray<b3Vector3>& getDeltaLinearVelocitiesCPU();
	b3AlignedObjectArray<b3Vector3>& getDeltaAngularVelocitiesCPU();
	b3AlignedObjectArray<unsigned int>& getBodyCountCPU();
	unsigned int getNumHostContactManifolds() const;
	unsigned int getTotalNumSplitBodiesCPU();
	b3AlignedObjectArray<unsigned int>& getOffsetSplitBodiesCPU();
};
#endif  //B3_GPU_JACOBI_CONTACT_SOLVER_H
