/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

package bullet.collision.broadphaseCollision

import bullet.collision.collisionDispatch.CollisionObjectWrapper
import bullet.collision.narrowPhaseCollision.PersistentManifold
import bullet.linearMath.DebugDraw

class DispatcherInfo {
    enum class DispatchFunc { Invalid, DISCRETE, CONTINUOUS }

    var timeStep = 0f
    var stepCount = 0
    var dispatchFunc = DispatchFunc.DISCRETE
    var timeOfImpact = 1f
    var useContinuous = true
    var debugDraw: DebugDraw? = null
    var enableSatConvex = false
    var enableSPU = true
    var useEpa = true
    var allowedCcdPenetration = 0.04f
    var useConvexConservativeDistanceUtil = false
    var convexConservativeDistanceThreshold = 0f
}

enum class DispatcherQueryType { Invalid, CONTACT_POINT_ALGORITHMS, CLOSEST_POINT_ALGORITHMS }

/** The btDispatcher interface class can be used in combination with broadphase to dispatch calculations for
 *  overlapping pairs.
 *  For example for pairwise collision detection, calculating contact points stored in btPersistentManifold or user
 *  callbacks (game logic). */
interface Dispatcher {

    fun findAlgorithm(body0Wrap: CollisionObjectWrapper,body1Wrap:CollisionObjectWrapper,sharedManifold: PersistentManifold,
                      queryType: DispatcherQueryType): CollisionAlgorithm

    virtual btPersistentManifold*	getNewManifold(const btCollisionObject* b0,const btCollisionObject* b1)=0;

    virtual void releaseManifold(btPersistentManifold* manifold)=0;

    virtual void clearManifold(btPersistentManifold* manifold)=0;

    virtual bool	needsCollision(const btCollisionObject* body0,const btCollisionObject* body1) = 0;

    virtual bool	needsResponse(const btCollisionObject* body0,const btCollisionObject* body1)=0;

    virtual void	dispatchAllCollisionPairs(btOverlappingPairCache* pairCache,const btDispatcherInfo& dispatchInfo,btDispatcher* dispatcher)  =0;

    virtual int getNumManifolds() const = 0;

    virtual btPersistentManifold* getManifoldByIndexInternal(int index) = 0;

    virtual	btPersistentManifold**	getInternalManifoldPointer() = 0;

    virtual	btPoolAllocator*	getInternalManifoldPool() = 0;

    virtual	const btPoolAllocator*	getInternalManifoldPool() const = 0;

    virtual	void* allocateCollisionAlgorithm(int size)  = 0;

    virtual	void freeCollisionAlgorithm(void* ptr) = 0;
}