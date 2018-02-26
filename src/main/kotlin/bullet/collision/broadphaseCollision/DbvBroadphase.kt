/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

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
import bullet.f
import bullet.linearMath.Vec3
import bullet.resize
import kotlin.math.max
import kotlin.math.min

// bvtBroadphase implementation by Nathanael Presson

//#define	DBVT_BP_PROFILE					0
////#define DBVT_BP_SORTPAIRS				1
//#define DBVT_BP_PREVENTFALSEUPDATE		0
//#define DBVT_BP_ACCURATESLEEPING		0
//#define DBVT_BP_ENABLE_BENCHMARK		0
val DBVT_BP_MARGIN = 0.05f
//
//#if DBVT_BP_PROFILE
//#define	DBVT_BP_PROFILING_RATE	256
//#include "LinearMath/btQuickprof.h"
//#endif

/** DbvtProxy   */
class DbvtProxy(aabbMin: Vec3, aabbMax: Vec3, userPtr: CollisionObject, collisionFilterGroup: Int, collisionFilterMask: Int) :
        BroadphaseProxy(aabbMin, aabbMax, userPtr, collisionFilterGroup, collisionFilterMask) {
    var leaf: DbvtNode? = null
    var links = Array<DbvtProxy?>(2, { null })
    var stage = 0
}

/** The DbvtBroadphase implements a broadphase using two dynamic AABB bounding volume hierarchies/trees (see Dbvt).
 *  One tree is used for static/non-moving objects, and another tree is used for dynamic objects.
 *  Objects can move from one tree to the other.
 *  This is a very fast broadphase, especially for very dynamic worlds where many objects are moving. Its insert/add
 *  and remove of objects is generally faster than the sweep and prune broadphases AxisSweep3 and _32BitAxisSweep3. */
class DbvtBroadphase(pairCache: OverlappingPairCache? = null) : BroadphaseInterface {

    // Config
    companion object {
        /* Dynamic set index	*/
        val DYNAMIC_SET = 0
        /* Fixed set index		*/
        val FIXED_SET = 1
        /* Number of stages		*/
        val STAGECOUNT = 2
    }

    /** Dbvt sets   */
    val sets = Array(2){ Dbvt() }
    /** Stages list */
    val stageRoots = Array<DbvtProxy?>(STAGECOUNT, { null })
    /** Pair cache  */
    var pairCache = pairCache ?: HashedOverlappingPairCache()
    /** Velocity prediction */
    var prediction = 0f
    /** Current stage   */
    var stageCurrent = 0
    /** % of fixed updates per frame    */
    var fUpdates = 1
    /** % of dynamic updates per frame  */
    var dUpdates = 0
    /** % of cleanup updates per frame  */
    var cUpdates = 10
    /** Number of pairs created */
    var newPairs = 1
    /** Fixed optimization left */
    var fixedLeft = 0
    /** Number of updates call  */
    var updatesCall = 0
    /** Number of updates done  */
    var updatesDone = 0
    /** updatesDone / updatesCall   */
    var updatesRatio = 0f
    /** Parse id    */
    var pId = 0
    /** Cleanup index   */
    var cId = 0
    /** Gen id  */
    var gId = 0
    /** Release pair cache on delete    */
    var releasePairCache = pairCache == null
    /** Defere dynamic/static collision to collide call */
    var deferedCollide = false
    /** Need to run cleanup?    */
    var needCleanup = true
    var rayTestStacks = ArrayList<DbvtNode>()

    // Methods
    fun collide(dispatcher: Dispatcher) {

        /* optimize				*/
        sets[0].optimizeIncremental(1 + (sets[0].leaves * dUpdates) / 100)
        if (fixedLeft != 0) {
            val count = 1 + (sets[1].leaves * fUpdates) / 100
            sets[1].optimizeIncremental(1 + (sets[1].leaves * fUpdates) / 100)
            fixedLeft = max(0, fixedLeft - count)
        }
        /* dynamic -> fixed set	*/
        stageCurrent = (stageCurrent + 1) % STAGECOUNT
        var current = stageRoots[stageCurrent]
        if (current != null) {
            do {
                val next = current!!.links[1]
                listRemove(current, stageRoots, current.stage)
                listAppend(current, stageRoots, STAGECOUNT)
                sets[0] remove current.leaf
                val curAabb = DbvtVolume.fromMM(current.aabbMin, current.aabbMax)
                current.leaf = sets[1].insert(curAabb, current)
                current.stage = STAGECOUNT
                current = next
            } while (current != null)
            fixedLeft = sets[1].leaves
            needCleanup = true
        }
        /* collide dynamics		*/
        run {
            val collider = DbvtTreeCollider(this)
            if (deferedCollide) {
                sets[0].collideTTpersistentStack(sets[0].root, sets[1].root, collider)
                sets[0].collideTTpersistentStack(sets[0].root, sets[0].root, collider)
            }
        }
        // clean up
        if (needCleanup) {
            val pairs = pairCache.overlappingPairArray
            if (pairs.size > 0) {
                var ni = min(pairs.size, max(newPairs, (pairs.size * cUpdates) / 100))
                var i = 0
                while (i < ni) {
                    val p = pairs[(cId + i) % pairs.size]!!
                    val pa = p.proxy0 as DbvtProxy
                    val pb = p.proxy1 as DbvtProxy
                    if (!(pa.leaf!!.volume intersect pb.leaf!!.volume)) {
                        pairCache.removeOverlappingPair(pa, pb, dispatcher)
                        --ni;--i
                    }
                    ++i
                }
                cId = if (pairs.size > 0) (cId + ni) % pairs.size else 0
            }
        }
        ++pId
        newPairs = 1
        needCleanup = false
        updatesRatio = if (updatesCall > 0) updatesDone / updatesCall.f else 0f
        updatesDone /= 2
        updatesCall /= 2
    }

    fun optimize() {
        sets[0].optimizeTopDown()
        sets[1].optimizeTopDown()
    }

    override fun createProxy(aabbMin: Vec3, aabbMax: Vec3, shapeType: Int, userPtr: CollisionObject, collisionFilterGroup: Int,
                             collisionFilterMask: Int, dispatcher: Dispatcher): BroadphaseProxy {

        val proxy = DbvtProxy(aabbMin, aabbMax, userPtr, collisionFilterGroup, collisionFilterMask)

        val aabb = DbvtVolume.fromMM(aabbMin, aabbMax)

        //bproxy->aabb			=	btDbvtVolume::FromMM(aabbMin,aabbMax);
        proxy.stage = stageCurrent
        proxy.uniqueId = ++gId
        proxy.leaf = sets[0].insert(aabb, proxy)
        listAppend(proxy, stageRoots, stageCurrent)
        if (!deferedCollide) {
            val collider = DbvtTreeCollider(this)
            collider.proxy = proxy
            sets[0].collideTV(sets[0].root, aabb, collider)
            sets[1].collideTV(sets[1].root, aabb, collider)
        }
        return proxy
    }

    override fun destroyProxy(proxy: BroadphaseProxy, dispatcher: Dispatcher) {
        val p = proxy as DbvtProxy
        sets[if (p.stage == STAGECOUNT) 1 else 0] remove p.leaf
        listRemove(p, stageRoots, p.stage)
        pairCache.removeOverlappingPairsContainingProxy(p, dispatcher)
        needCleanup = true
    }

    override fun setAabb(proxy: BroadphaseProxy, aabbMin: Vec3, aabbMax: Vec3, dispatcher: Dispatcher) {
        val p = proxy as DbvtProxy
        val aabb = DbvtVolume.fromMM(aabbMin, aabbMax)

        var doCollide = false
        if (p.stage == STAGECOUNT) {
            /* fixed -> dynamic set	*/
            sets[1] remove p.leaf
            p.leaf = sets[0].insert(aabb, p)
            doCollide = true
        } else {
            /* dynamic set				*/
            ++updatesCall
            if (p.leaf!!.volume intersect aabb) {
                /* Moving				*/
                val delta = aabbMin - p.aabbMin
                val velocity = Vec3(((p.aabbMax - p.aabbMin) / 2f) * prediction)
                if (delta[0] < 0) velocity[0] = -velocity[0]
                if (delta[1] < 0) velocity[1] = -velocity[1]
                if (delta[2] < 0) velocity[2] = -velocity[2]
                if (sets[0].update(p.leaf!!, aabb, velocity, DBVT_BP_MARGIN)) {
                    ++updatesDone
                    doCollide = true
                }
            } else {
                /* Teleporting			*/
                sets[0].update(p.leaf!!, aabb)
                ++updatesDone
                doCollide = true
            }
        }
        listRemove(p, stageRoots, p.stage)
        p.aabbMin put aabbMin
        p.aabbMax put aabbMax
        p.stage = stageCurrent
        listAppend(p, stageRoots, stageCurrent)
        if (doCollide) {
            needCleanup = true
            if (!deferedCollide) {
                val collider = DbvtTreeCollider(this)
                sets[1].collideTTpersistentStack(sets[1].root, p.leaf, collider)
                sets[0].collideTTpersistentStack(sets[0].root, p.leaf, collider)
            }
        }
    }

    override fun rayTest(rayFrom: Vec3, rayTo: Vec3, rayCallback: BroadphaseRayCallback, aabbMin: Vec3, aabbMax: Vec3) {
        val callback = BroadphaseRayTester(rayCallback)
        val stack = rayTestStacks

        sets[0].rayTestInternal(sets[0].root,
                rayFrom,
                rayTo,
                rayCallback.rayDirectionInverse,
                rayCallback.signs,
                rayCallback.lambdaMax,
                aabbMin,
                aabbMax,
                stack,
                callback)

        sets[1].rayTestInternal(sets[1].root,
                rayFrom,
                rayTo,
                rayCallback.rayDirectionInverse,
                rayCallback.signs,
                rayCallback.lambdaMax,
                aabbMin,
                aabbMax,
                stack,
                callback)
    }

    override fun aabbTest(aabbMin: Vec3, aabbMax: Vec3, callback: BroadphaseAabbCallback) {
        val callback = BroadphaseAabbTester(callback)
        val bounds = DbvtVolume.fromMM(aabbMin, aabbMax)
        //process all children, that overlap with  the given AABB bounds
        sets[0].collideTV(sets[0].root, bounds, callback)
        sets[1].collideTV(sets[1].root, bounds, callback)
    }

    override fun getAabb(proxy: BroadphaseProxy, aabbMin: Vec3, aabbMax: Vec3) {
        val proxy = proxy as DbvtProxy
        aabbMin put proxy.aabbMin
        aabbMax put proxy.aabbMax
    }

    override fun calculateOverlappingPairs(dispatcher: Dispatcher) {
        collide(dispatcher)
        performDeferredRemoval(dispatcher)
    }

    override val overlappingPairCache get() = pairCache

    override fun getBroadphaseAabb(aabbMin: Vec3, aabbMax: Vec3) {
        val bounds = when {
            !sets[0].empty() -> {
                if (!sets[1].empty())
                    DbvtVolume().also { sets[0].root!!.volume.merge(sets[1].root!!.volume, it) }
                else sets[0].root!!.volume
            }
            !sets[1].empty() -> sets[1].root!!.volume
            else -> DbvtVolume.fromCR(Vec3(), 0f)
        }
        aabbMin put bounds.min
        aabbMax put bounds.max
    }

    override fun printStats() = Unit

    /** reset broadphase internal structures, to ensure determinism/reproducability */
    override fun resetPool(dispatcher: Dispatcher) {
        val totalObjects = sets[0].leaves + sets[1].leaves
        if (totalObjects == 0) {
            //reset internal dynamic tree data structures
            sets[0].clear()
            sets[1].clear()

            deferedCollide = false
            needCleanup = true
            stageCurrent = 0
            fixedLeft = 0
            fUpdates = 1
            dUpdates = 0
            cUpdates = 10
            newPairs = 1
            updatesCall = 0
            updatesDone = 0
            updatesRatio = 0f

            gId = 0
            pId = 0
            cId = 0
            for (i in 0..STAGECOUNT) stageRoots[i] = null
        }
    }

    fun performDeferredRemoval(dispatcher: Dispatcher) {

        if (!pairCache.hasDeferredRemoval) return

        val overlappingPairArray = pairCache.overlappingPairArray

        //perform a sort, to find duplicates and to sort 'invalid' pairs to the end
        overlappingPairArray quickSort ::BroadphasePairSortPredicate

        var invalidPair = 0

        var previousPair = BroadphasePair()

        for(i in 0 until overlappingPairArray.size) {

            val pair = overlappingPairArray[i]!!

            val isDuplicate = pair === previousPair

            previousPair = pair

            var needsRemoval: Boolean

            if (!isDuplicate) {
                //important to perform AABB check that is consistent with the broadphase
                val pa = pair.proxy0 as DbvtProxy
                val pb = pair.proxy1 as DbvtProxy
                val hasOverlap = pa.leaf!!.volume intersect pb.leaf!!.volume
                needsRemoval = !hasOverlap
            } else {
                needsRemoval = true //remove duplicate
                assert(pair.algorithm == null) //should have no algorithm
            }

            if (needsRemoval) {
                pairCache.cleanOverlappingPair(pair, dispatcher)
                pair.proxy0 = null
                pair.proxy1 = null
                invalidPair++
            }
        }
        //perform a sort, to sort 'invalid' pairs to the end
        overlappingPairArray quickSort ::BroadphasePairSortPredicate
        overlappingPairArray.resize(overlappingPairArray.size - invalidPair)
    }

    /** This setAabbForceUpdate is similar to setAabb but always forces the aabb update.
     *  It is not part of the BroadphaseInterface but specific to DbvtBroadphase.
     *  It bypasses certain optimizations that prevent aabb updates (when the aabb shrinks), see
     *  http://code.google.com/p/bullet/issues/detail?id=223    */
    fun setAabbForceUpdate(absProxy: BroadphaseProxy, aabbMin: Vec3, aabbMax: Vec3, dispatcher: Dispatcher) {
        val proxy = absProxy as DbvtProxy
        val aabb = DbvtVolume.fromMM(aabbMin, aabbMax)
        var doCollide = false
        if (proxy.stage == STAGECOUNT) {/* fixed -> dynamic set	*/
            sets[1] remove proxy.leaf
            proxy.leaf = sets[0].insert(aabb, proxy)
            doCollide = true
        } else {/* dynamic set				*/
            ++updatesCall
            /* Teleporting			*/
            sets[0].update(proxy.leaf!!, aabb)
            ++updatesDone
            doCollide = true
        }
        listRemove(proxy, stageRoots, proxy.stage)
        proxy.aabbMin = aabbMin
        proxy.aabbMax = aabbMax
        proxy.stage = stageCurrent
        listAppend(proxy, stageRoots, stageCurrent)
        if (doCollide) {
            needCleanup = true
            if (!deferedCollide) {
                val collider = DbvtTreeCollider(this)
                sets[1].collideTTpersistentStack(sets[1].root, proxy.leaf, collider)
                sets[0].collideTTpersistentStack(sets[0].root, proxy.leaf, collider)
            }
        }
    }

    fun benchmark(broadphase: BroadphaseInterface) = Unit
}

fun listAppend(item: DbvtProxy, list: Array<DbvtProxy?>, ptr: Int) {
    item.links[0] = null
    item.links[1] = list[ptr]
    list[ptr]?.links?.set(0, item)
    list[ptr] = item
}

fun listRemove(item: DbvtProxy, list: Array<DbvtProxy?>, ptr: Int) {
    if (item.links[0] != null) item.links[0]!!.links[1] = item.links[1] else list[ptr] = item.links[1]
    if (item.links[1] != null) item.links[1]!!.links[0] = item.links[0]
}

fun listCount(root: DbvtProxy?): Int {
    var n = 0
    var root = root
    while (root != null) {
        ++n; root = root.links[1]
    }
    return n
}

//fun clear(T& value)        {
//    static const struct ZeroDummy : T {} zerodummy
//            value = zerodummy
//}

//
// Colliders
//

/* Tree collider	*/
class DbvtTreeCollider(p: DbvtBroadphase) : Dbvt.Collide {
    var pbp = p
    var proxy: DbvtProxy? = null
    override fun process(leaf0: DbvtNode, leaf1: DbvtNode) {
        if (leaf0 !== leaf1) {
            val pa = leaf0.data as DbvtProxy
            val pb = leaf1.data as DbvtProxy
            pbp.pairCache.addOverlappingPair(pa, pb)
            ++pbp.newPairs
        }
    }

    override infix fun process(node: DbvtNode) = process(node, proxy!!.leaf!!)
}

class BroadphaseRayTester(val rayCallback: BroadphaseRayCallback) : Dbvt.Collide {
    override fun process(node: DbvtNode) {
        val proxy = node.data as DbvtProxy
        rayCallback process proxy
    }
}

class BroadphaseAabbTester(val aabbCallback: BroadphaseAabbCallback) : Dbvt.Collide {
    override fun process(node: DbvtNode) {
        val proxy = node.data as DbvtProxy
        aabbCallback process proxy
    }
}