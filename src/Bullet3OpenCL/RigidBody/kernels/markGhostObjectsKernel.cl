#pragma OPENCL EXTENSION cl_khr_byte_addressable_store : enable
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3UpdateAabbs.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3Collidable.h"

__kernel void markGhostObjectPairsKernel(__global int4* pairs, __global int* collisionFlags, int numPairs, int numObjects)
{
	int pairId = get_global_id(0);
	if (pairId < numPairs)
	{
		// Use the w component of the collision pair to store non-default collision types/attributes
		// This kernel marks a pair as being ignored by the collision handling if one of the involved objects is a ghost object
		if (pairs[pairId].x < numObjects)
		{
			if (collisionFlags[pairs[pairId].x] & CF_GHOST_OBJECT)
				pairs[pairId].w = CF_GHOST_OBJECT;
		}

		if (pairs[pairId].y < numObjects)
		{
			if (collisionFlags[pairs[pairId].y] & CF_GHOST_OBJECT)
				pairs[pairId].w = CF_GHOST_OBJECT;
		}
	}
}