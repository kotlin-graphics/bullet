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

package bullet.collision.collisionDispatch

import bullet.collision.broadphaseCollision.*
import bullet.collision.narrowPhaseCollision.PersistentManifold
import bullet.collision.narrowPhaseCollision.gContactBreakingThreshold
import bullet.has
import bullet.hasnt
import bullet.pop
import bullet.swapLastAt
import kotlin.math.min
import bullet.collision.broadphaseCollision.BroadphaseNativeTypes as BNT
import bullet.collision.broadphaseCollision.DispatcherQueryType as DQT
import bullet.collision.collisionDispatch.CollisionDispatcher.DispatcherFlags as DF

var numManifold = 0

/** user can override this nearcallback for collision filtering and more finegrained control over collision detection   */
typealias NearCallback = (BroadphasePair, CollisionDispatcher, DispatcherInfo) -> Unit

/** CollisionDispatcher supports algorithms that handle ConvexConvex and ConvexConcave collision pairs.
 *  Time of Impact, Closest Points and Penetration Depth.   */
class CollisionDispatcher(val collisionConfiguration: CollisionConfiguration) : Dispatcher {

    var dispatcherFlags = DF.USE_RELATIVE_CONTACT_BREAKING_THRESHOLD.i

    var manifolds = arrayListOf<PersistentManifold>()

    val defaultManifoldResult = ManifoldResult()

    var nearCallback = ::defaultNearCallback

//    btPoolAllocator*    m_collisionAlgorithmPoolAllocator TODO

//    btPoolAllocator*    m_persistentManifoldPoolAllocator

    val doubleDispatchContactPoints = Array(BNT.MAX_BROADPHASE_COLLISION_TYPES.i, { i ->
        Array(BNT.MAX_BROADPHASE_COLLISION_TYPES.i, { j -> collisionConfiguration.getCollisionAlgorithmCreateFunc(i, j) })
    })
    val doubleDispatchClosestPoints = Array(BNT.MAX_BROADPHASE_COLLISION_TYPES.i, { i ->
        Array(BNT.MAX_BROADPHASE_COLLISION_TYPES.i, { j -> collisionConfiguration.getClosestPointsAlgorithmCreateFunc(i, j) })
    })

    enum class DispatcherFlags { STATIC_STATIC_REPORTED, USE_RELATIVE_CONTACT_BREAKING_THRESHOLD, DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION;

        val i = 1 shl ordinal
    }

    /** registerCollisionCreateFunc allows registration of custom/alternative collision create functions    */
    fun registerCollisionCreateFunc(proxyType0: Int, proxyType1: Int, createFunc: CollisionAlgorithmCreateFunc) =   // TODO useful?
            doubleDispatchContactPoints[proxyType0].set(proxyType1, createFunc)

    fun registerClosestPointsCreateFunc(proxyType0: Int, proxyType1: Int, createFunc: CollisionAlgorithmCreateFunc) =
            doubleDispatchClosestPoints[proxyType0].set(proxyType1, createFunc)

    override val numManifolds get() = manifolds.size

    override val internalManifoldPointer get() = manifolds.takeIf { it.isNotEmpty() }

    override fun getManifoldByIndexInternal(index: Int) = manifolds[index]

    override fun getNewManifold(body0: CollisionObject, body1: CollisionObject): PersistentManifold {

        numManifold++
        // optional relative contact breaking threshold, turned on by default (use setDispatcherFlags to switch off feature for improved performance)
        val contactBreakingThreshold = if (dispatcherFlags has DF.USE_RELATIVE_CONTACT_BREAKING_THRESHOLD.i)
            min(body0.collisionShape!!.getContactBreakingThreshold(gContactBreakingThreshold), body1.collisionShape!!.getContactBreakingThreshold(gContactBreakingThreshold))
        else gContactBreakingThreshold

        val contactProcessingThreshold = min(body0.contactProcessingThreshold, body1.contactProcessingThreshold)

//        void* mem = m_persistentManifoldPoolAllocator->allocate( sizeof( btPersistentManifold ) ) TODO check
//        if (NULL == mem)
//        {
//            //we got a pool memory overflow, by default we fallback to dynamically allocate memory. If we require a contiguous contact pool then assert.
//            if ((m_dispatcherFlags&CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION)==0)
//            {
//                mem = btAlignedAlloc(sizeof(btPersistentManifold),16)
//            } else
//            {
//                btAssert(0)
//                //make sure to increase the m_defaultMaxPersistentManifoldPoolSize in the btDefaultCollisionConstructionInfo/btDefaultCollisionConfiguration
//                return 0
//            }
//        }
        val manifold = PersistentManifold(body0, body1, 0, contactBreakingThreshold, contactProcessingThreshold)
        manifold.index1a = manifolds.size
        manifolds.add(manifold)

        return manifold
    }

    override fun releaseManifold(manifold: PersistentManifold) {
        numManifold--

        //printf("releaseManifold: gNumManifold %d\n",gNumManifold);
        clearManifold(manifold)

        val findIndex = manifold.index1a
        assert(findIndex < manifolds.size)
        manifolds.swapLastAt(findIndex)
        manifolds[findIndex].index1a = findIndex
        manifolds.pop()
    }

    override fun clearManifold(manifold: PersistentManifold) = manifold.clearManifold()

    override fun findAlgorithm(body0Wrap: CollisionObjectWrapper, body1Wrap: CollisionObjectWrapper, sharedManifold: PersistentManifold?,
                               queryType: DQT): CollisionAlgorithm {
        val a = body0Wrap.collisionShape.shapeType.i
        val b = body1Wrap.collisionShape.shapeType.i
        val ci = CollisionAlgorithmConstructionInfo(this, sharedManifold)
        return when (queryType) {
            DQT.CONTACT_POINT_ALGORITHMS -> doubleDispatchContactPoints[a][b].createCollisionAlgorithm(ci, body0Wrap, body1Wrap)
            else -> doubleDispatchClosestPoints[a][b].createCollisionAlgorithm(ci, body0Wrap, body1Wrap)
        }
    }

    override fun needsCollision(body0: CollisionObject?, body1: CollisionObject?): Boolean {
        body0!!
        body1!!

        var needsCollision = true

        if (dispatcherFlags hasnt DF.STATIC_STATIC_REPORTED.i)
        //broadphase filtering already deals with this
            if (body0.isStaticOrKinematicObject && body1.isStaticOrKinematicObject) {
                dispatcherFlags = dispatcherFlags or DF.STATIC_STATIC_REPORTED.i
                println("warning CollisionDispatcher::needsCollision: static-static collision!")
            }

        if (!body0.isActive && !body1.isActive)
            needsCollision = false
        else if (!body0.checkCollideWith(body1) || !body1.checkCollideWith(body0))
            needsCollision = false

        return needsCollision
    }

    override fun needsResponse(body0: CollisionObject, body1: CollisionObject): Boolean {
        // here you can do filtering
        val hasResponse = body0.hasContactResponse && body1.hasContactResponse
        // no response between two static/kinematic bodies:
        return hasResponse && !body0.isStaticOrKinematicObject || !body1.isStaticOrKinematicObject
    }

    override fun dispatchAllCollisionPairs(pairCache: OverlappingPairCache, dispatchInfo: DispatcherInfo, dispatcher: Dispatcher) {
        val collisionCallback = CollisionPairCallback(dispatchInfo, this)
        pairCache.processAllOverlappingPairs(collisionCallback, dispatcher)
    }

    companion object {
        /** by default, Bullet will use this near callback  */
        fun defaultNearCallback(collisionPair: BroadphasePair, dispatcher: CollisionDispatcher, dispatchInfo: DispatcherInfo) {
            val colObj0 = collisionPair.proxy0!!.clientObject as CollisionObject
            val colObj1 = collisionPair.proxy1!!.clientObject as CollisionObject

            if (dispatcher.needsCollision(colObj0, colObj1)) {
                val obj0Wrap = CollisionObjectWrapper(null, colObj0.collisionShape, colObj0, colObj0.getWorldTransform(), -1, -1)
                val obj1Wrap = CollisionObjectWrapper(null, colObj1.collisionShape, colObj1, colObj1.getWorldTransform(), -1, -1)

                //dispatcher will keep algorithms persistent in the collision pair
                if (collisionPair.algorithm == null)
                    collisionPair.algorithm = dispatcher.findAlgorithm(obj0Wrap, obj1Wrap, null, DQT.CONTACT_POINT_ALGORITHMS)

                collisionPair.algorithm?.let {
                    val contactPointResult = ManifoldResult(obj0Wrap, obj1Wrap)

                    if (dispatchInfo.dispatchFunc == DispatcherInfo.DispatchFunc.DISCRETE)
                    //discrete collision detection query
                        it.processCollision(obj0Wrap, obj1Wrap, dispatchInfo, contactPointResult)
                    else {
                        //continuous collision detection query, time of impact (toi)
                        val toi = it.calculateTimeOfImpact(colObj0, colObj1, dispatchInfo, contactPointResult)
                        if (dispatchInfo.timeOfImpact > toi) dispatchInfo.timeOfImpact = toi
                    }
                }
            }
        }
    }

    override fun allocateCollisionAlgorithm(size: Int): Any? = null
    override fun freeCollisionAlgorithm(ptr: Any?) = Unit
}

/** interface for iterating all overlapping collision pairs, no matter how those pairs are stored (array, set, map etc)
 *  this is useful for the collision dispatcher.    */
class CollisionPairCallback(val dispatchInfo: DispatcherInfo, val dispatcher: CollisionDispatcher) : OverlapCallback {
    override fun processOverlap(pair: BroadphasePair): Boolean {
        dispatcher.nearCallback(pair, dispatcher, dispatchInfo)
        return false
    }
}