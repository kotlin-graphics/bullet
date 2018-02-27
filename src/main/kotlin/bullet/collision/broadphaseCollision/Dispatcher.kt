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

import bullet.collision.collisionDispatch.CollisionObject
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
    var deterministicOverlappingPairs = true
}

enum class DispatcherQueryType { Invalid, CONTACT_POINT_ALGORITHMS, CLOSEST_POINT_ALGORITHMS }

/** The btDispatcher interface class can be used in combination with broadphase to dispatch calculations for
 *  overlapping pairs.
 *  For example for pairwise collision detection, calculating contact points stored in btPersistentManifold or user
 *  callbacks (game logic). */
interface Dispatcher {

    fun findAlgorithm(body0Wrap: CollisionObjectWrapper, body1Wrap: CollisionObjectWrapper, sharedManifold: PersistentManifold?,
                      queryType: DispatcherQueryType): CollisionAlgorithm?

    fun getNewManifold(body0: CollisionObject, body1: CollisionObject): PersistentManifold

    fun releaseManifold(manifold: PersistentManifold)

    fun clearManifold(manifold: PersistentManifold)

    fun needsCollision(body0: CollisionObject?, body1: CollisionObject?): Boolean

    fun needsResponse(body0: CollisionObject, body1: CollisionObject): Boolean

    fun dispatchAllCollisionPairs(pairCache: OverlappingPairCache, dispatchInfo: DispatcherInfo, dispatcher: Dispatcher)

    val numManifolds: Int

    fun getManifoldByIndexInternal(index: Int): PersistentManifold

    val internalManifoldPointer: ArrayList<PersistentManifold>? // TODO virtual	btPersistentManifold**	getInternalManifoldPointer() = 0;

//    virtual    btPoolAllocator*    getInternalManifoldPool() = 0;

//    virtual    const btPoolAllocator*    getInternalManifoldPool() const = 0;

    fun allocateCollisionAlgorithm(size: Int): Any?

    fun freeCollisionAlgorithm(ptr: Any?) // TODO useless, remove
}