/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

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

import bullet.BT_PROFILE
import bullet.collision.broadphaseCollision.*
import bullet.collision.collisionShapes.CompoundShape
import bullet.collision.narrowPhaseCollision.PersistentManifold
import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import bullet.linearMath.testAabbAgainstAabb2
import bullet.linearMath.transformAabb
import bullet.resize
import bullet.collision.broadphaseCollision.DispatcherQueryType as Dqt

var gCompoundCompoundChildShapePairCallback: ShapePairCallback? = null

/** CompoundCompoundCollisionAlgorithm  supports collision between two CompoundCollisionShape shapes */
class CompoundCompoundCollisionAlgorithm(ci: CollisionAlgorithmConstructionInfo, val body0Wrap: CollisionObjectWrapper,
                                         val body1Wrap: CollisionObjectWrapper, isSwapped: Boolean) :
        CompoundCollisionAlgorithm(ci, body0Wrap, body1Wrap, isSwapped) {

    val childCollisionAlgorithmCache = HashedSimplePairCache()
    val removePairs = ArrayList<SimplePair>()
    // to keep track of changes, so that childAlgorithm array can be updated
    var compoundShapeRevision0 = (body0Wrap.collisionShape as CompoundShape).updateRevision
    var compoundShapeRevision1 = (body1Wrap.collisionShape as CompoundShape).updateRevision

    override fun removeChildAlgorithms() {
        val pairs = childCollisionAlgorithmCache.overlappingPairArray.data
        val numChildren = pairs.size
        for (i in 0 until numChildren)
            pairs[i]?.userPointer?.let { dispatcher!!.freeCollisionAlgorithm(it as CollisionAlgorithm) }
        childCollisionAlgorithmCache.removeAllPairs()
    }

    override fun processCollision(body0Wrap: CollisionObjectWrapper, body1Wrap: CollisionObjectWrapper, dispatchInfo: DispatcherInfo, resultOut: ManifoldResult) {

        val col0ObjWrap = body0Wrap
        val col1ObjWrap = body1Wrap

        assert(col0ObjWrap.collisionShape.isCompound)
        assert(col1ObjWrap.collisionShape.isCompound)
        val compoundShape0 = col0ObjWrap.collisionShape as CompoundShape
        val compoundShape1 = col1ObjWrap.collisionShape as CompoundShape

        val tree0 = compoundShape0.dynamicAabbTree
        val tree1 = compoundShape1.dynamicAabbTree
        if (tree0 == null || tree1 == null)
            return super.processCollision(body0Wrap, body1Wrap, dispatchInfo, resultOut)
        // CompoundShape might have changed: make sure the internal child collision algorithm caches are still valid
        if (compoundShape0.updateRevision != compoundShapeRevision0 || compoundShape1.updateRevision != compoundShapeRevision1) {
            ///clear all
            removeChildAlgorithms()
            compoundShapeRevision0 = compoundShape0.updateRevision
            compoundShapeRevision1 = compoundShape1.updateRevision
        }
        /*  we need to refresh all contact manifolds
            note that we should actually recursively traverse all children, CompoundShape can nested more then 1 level
            deep
            so we should add a 'refreshManifolds' in the CollisionAlgorithm */
        run {
            val manifoldArray = ArrayList<PersistentManifold>()
            val pairs = childCollisionAlgorithmCache.overlappingPairArray.data
            for (i in 0 until pairs.size)
                pairs[i]?.userPointer?.let {
                    val algo = it as CollisionAlgorithm
                    algo.getAllContactManifolds(manifoldArray)
                    manifoldArray.filter { it.numContacts != 0 }.forEach {
                        with(resultOut) {
                            manifold = it
                            refreshContactPoints()
                            manifold = null
                        }
                    }
                    manifoldArray.clear()
                }
        }

        val callback = CompoundCompoundLeafCallback(col0ObjWrap, col1ObjWrap, dispatcher!!, dispatchInfo, resultOut,
                childCollisionAlgorithmCache, sharedManifold!!)

        val xform = col0ObjWrap.worldTransform.inverse() * col1ObjWrap.worldTransform
        mycollideTT(tree0.root, tree1.root, xform, callback, resultOut.closestPointDistanceThreshold)

        //printf("#compound-compound child/leaf overlap =%d                      \r",callback.m_numOverlapPairs);

        // remove non-overlapping child pairs

        assert(removePairs.isEmpty())

        // iterate over all children, perform an AABB check inside ProcessChildShape
        val pairs = childCollisionAlgorithmCache.overlappingPairArray.data

        // should be clear, reuse
        assert(manifoldArray.isEmpty())

        val aabbMin0 = Vec3()
        val aabbMax0 = Vec3()
        val aabbMin1 = Vec3()
        val aabbMax1 = Vec3()

        for (i in 0 until pairs.size)
            pairs[i]?.let { pair ->
                pair.userPointer?.let {

                    val algo = it as CollisionAlgorithm

                    val childShape0 = compoundShape0.getChildShape(pair.indexA)
                    val childTrans0 = compoundShape0.getChildTransform(pair.indexA)
                    val newChildWorldTrans0 = col0ObjWrap.worldTransform * childTrans0
                    childShape0.getAabb(newChildWorldTrans0, aabbMin0, aabbMax0)

                    val thresholdVec = Vec3(resultOut.closestPointDistanceThreshold)
                    aabbMin0 -= thresholdVec
                    aabbMax0 += thresholdVec

                    val childShape1 = compoundShape1.getChildShape(pair.indexB)
                    val childTrans1 = compoundShape1.getChildTransform(pair.indexB)
                    val newChildWorldTrans1 = col1ObjWrap.worldTransform * childTrans1
                    childShape1.getAabb(newChildWorldTrans1, aabbMin1, aabbMax1)

                    aabbMin1 -= thresholdVec
                    aabbMax1 += thresholdVec

                    if (!testAabbAgainstAabb2(aabbMin0, aabbMax0, aabbMin1, aabbMax1)) {
                        dispatcher!!.freeCollisionAlgorithm(algo)
                        removePairs += SimplePair(pair.indexA, pair.indexB)
                    }
                }

            }
        removePairs.forEach { childCollisionAlgorithmCache.removeOverlappingPair(it.indexA, it.indexB) }
        removePairs.clear()
    }

    override fun calculateTimeOfImpact(body0: CollisionObject, body1: CollisionObject, dispatchInfo: DispatcherInfo, resultOut: ManifoldResult): Float {
        assert(false)
        return 0f
    }

    override fun getAllContactManifolds(manifoldArray: ArrayList<PersistentManifold>) {
        val pairs = childCollisionAlgorithmCache.overlappingPairArray
        for (i in 0 until pairs.size)
            pairs[i]?.userPointer?.let { (it as CollisionAlgorithm).getAllContactManifolds(manifoldArray) }
    }

    class CreateFunc : CollisionAlgorithmCreateFunc() {
        override fun createCollisionAlgorithm(info: CollisionAlgorithmConstructionInfo, body0Wrap: CollisionObjectWrapper,
                                              body1Wrap: CollisionObjectWrapper) =
                CompoundCompoundCollisionAlgorithm(info, body0Wrap, body1Wrap, false)
    }

    class SwappedCreateFunc : CollisionAlgorithmCreateFunc() {
        override fun createCollisionAlgorithm(info: CollisionAlgorithmConstructionInfo, body0Wrap: CollisionObjectWrapper,
                                              body1Wrap: CollisionObjectWrapper) =
                CompoundCompoundCollisionAlgorithm(info, body0Wrap, body1Wrap, true)
    }

    companion object {

        fun mycollideTT(root0: DbvtNode?, root1: DbvtNode?, xform: Transform, callback: CompoundCompoundLeafCallback,
                        distanceThreshold: Float) {

            if (root0 != null && root1 != null) {
                var depth = 1
                var treshold = Dbvt.DOUBLE_STACKSIZE - 4
                val stkStack = ArrayList<Dbvt.StkNN?>()
                stkStack.resize(Dbvt.DOUBLE_STACKSIZE)
                stkStack[0] = Dbvt.StkNN(root0, root1)
                do {
                    val p = stkStack[--depth]!!
                    val pa = p.a!!
                    val pb = p.b!!
                    if (myIntersect(pa.volume, pb.volume, xform, distanceThreshold)) {
                        if (depth > treshold) {
                            stkStack.resize(stkStack.size * 2)
                            treshold = stkStack.size - 4
                        }
                        when {
                            pa.isInternal -> when {
                                pb.isInternal -> {
                                    stkStack[depth++] = Dbvt.StkNN(pa.childs[0], pb.childs[0])
                                    stkStack[depth++] = Dbvt.StkNN(pa.childs[1], pb.childs[0])
                                    stkStack[depth++] = Dbvt.StkNN(pa.childs[0], pb.childs[1])
                                    stkStack[depth++] = Dbvt.StkNN(pa.childs[1], pb.childs[1])
                                }
                                else -> {
                                    stkStack[depth++] = Dbvt.StkNN(pa.childs[0], pb)
                                    stkStack[depth++] = Dbvt.StkNN(pa.childs[1], pb)
                                }
                            }
                            else -> when {
                                pb.isInternal -> {
                                    stkStack[depth++] = Dbvt.StkNN(pa, pb.childs[0])
                                    stkStack[depth++] = Dbvt.StkNN(pa, pb.childs[1])
                                }
                                else -> callback.process(pa, pb)
                            }
                        }
                    }
                } while (depth != 0)
            }
        }

        fun myIntersect(a: DbvtAabbMm, b: DbvtAabbMm, xform: Transform, distanceThreshold: Float): Boolean {
            val newMin = Vec3()
            val newMax = Vec3()
            transformAabb(b.min, b.max, 0f, xform, newMin, newMax)
            newMin -= distanceThreshold
            newMax += distanceThreshold
            val newB = DbvtAabbMm.fromMM(newMin, newMax)
            return a intersect newB
        }
    }
}

class CompoundCompoundLeafCallback(val compound1ObjWrap: CollisionObjectWrapper, val compound0ObjWrap: CollisionObjectWrapper,
                                   val dispatcher: Dispatcher, val dispatchInfo: DispatcherInfo, val resultOut: ManifoldResult,
                                   val childAlgorithmsCache: HashedSimplePairCache, val sharedManifold: PersistentManifold)
    : Dbvt.Collide {

    var numOverlapPairs = 0


    override fun process(leaf0: DbvtNode, leaf1: DbvtNode) {
        BT_PROFILE("CompoundCompoundLeafCallback::Process")
        numOverlapPairs++

        val childIndex0 = leaf0.dataAsInt
        val childIndex1 = leaf1.dataAsInt

        assert(childIndex0 >= 0)
        assert(childIndex1 >= 0)

        val compoundShape0 = compound0ObjWrap.collisionShape as CompoundShape
        assert(childIndex0 < compoundShape0.numChildShapes)

        val compoundShape1 = compound1ObjWrap.collisionShape as CompoundShape
        assert(childIndex1 < compoundShape1.numChildShapes)

        val childShape0 = compoundShape0.getChildShape(childIndex0)
        val childShape1 = compoundShape1.getChildShape(childIndex1)
        //backup
        val orgTrans0 = compound0ObjWrap.worldTransform
        val childTrans0 = compoundShape0.getChildTransform(childIndex0)
        val newChildWorldTrans0 = orgTrans0 * childTrans0

        val orgTrans1 = compound1ObjWrap.worldTransform
        val childTrans1 = compoundShape1.getChildTransform(childIndex1)
        val newChildWorldTrans1 = orgTrans1 * childTrans1
        //perform an AABB check first
        val aabbMin0 = Vec3()
        val aabbMax0 = Vec3()
        val aabbMin1 = Vec3()
        val aabbMax1 = Vec3()
        childShape0.getAabb(newChildWorldTrans0, aabbMin0, aabbMax0)
        childShape1.getAabb(newChildWorldTrans1, aabbMin1, aabbMax1)

        val thresholdVec = Vec3(resultOut.closestPointDistanceThreshold)

        aabbMin0 -= thresholdVec
        aabbMax0 += thresholdVec

        gCompoundCompoundChildShapePairCallback?.apply { if (!invoke(childShape0, childShape1)) return }

        if (testAabbAgainstAabb2(aabbMin0, aabbMax0, aabbMin1, aabbMax1)) {
            val compoundWrap0 = CollisionObjectWrapper(compound0ObjWrap, childShape0, compound0ObjWrap.collisionObject!!,
                    newChildWorldTrans0, -1, childIndex0)
            val compoundWrap1 = CollisionObjectWrapper(compound1ObjWrap, childShape1, compound1ObjWrap.collisionObject!!,
                    newChildWorldTrans1, -1, childIndex1)

            val colAlgo = when {
                resultOut.closestPointDistanceThreshold > 0 ->
                    dispatcher.findAlgorithm(compoundWrap0, compoundWrap1, null, Dqt.CLOSEST_POINT_ALGORITHMS)
                else -> childAlgorithmsCache.findPair(childIndex0, childIndex1)?.let { it.userPointer as CollisionAlgorithm }
                        ?: dispatcher.findAlgorithm(compoundWrap0, compoundWrap1, sharedManifold, Dqt.CONTACT_POINT_ALGORITHMS)
                                .also { childAlgorithmsCache.addOverlappingPair(childIndex0, childIndex1)!!.userPointer = it }
            }!!

            val tmpWrap0 = resultOut.body0Wrap
            val tmpWrap1 = resultOut.body1Wrap

            resultOut.body0Wrap = compoundWrap0
            resultOut.body1Wrap = compoundWrap1

            resultOut.setShapeIdentifiersA(-1, childIndex0)
            resultOut.setShapeIdentifiersB(-1, childIndex1)

            colAlgo.processCollision(compoundWrap0, compoundWrap1, dispatchInfo, resultOut)

            resultOut.body0Wrap = tmpWrap0
            resultOut.body1Wrap = tmpWrap1
        }
    }
}