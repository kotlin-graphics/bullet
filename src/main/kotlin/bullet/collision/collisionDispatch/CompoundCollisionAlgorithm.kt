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
import bullet.collision.collisionShapes.CollisionShape
import bullet.collision.collisionShapes.CompoundShape
import bullet.collision.narrowPhaseCollision.PersistentManifold
import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import bullet.linearMath.testAabbAgainstAabb2
import bullet.resize

typealias ShapePairCallback = (CollisionShape, CollisionShape) -> Boolean
var gCompoundChildShapePairCallback: ShapePairCallback? = null

/** CompoundCollisionAlgorithm  supports collision between CompoundCollisionShapes and other collision shapes */
open class CompoundCollisionAlgorithm(ci: CollisionAlgorithmConstructionInfo, body0Wrap: CollisionObjectWrapper,
                                 body1Wrap: CollisionObjectWrapper, val isSwapped: Boolean) :
        ActivatingCollisionAlgorithm(ci, body0Wrap, body1Wrap) {

    val stack2 = ArrayList<DbvtNode>()
    val manifoldArray = ArrayList<PersistentManifold>()

    val childCollisionAlgorithms = ArrayList<CollisionAlgorithm?>()

    val sharedManifold = ci.manifold
    var ownsManifold = false

    /** to keep track of changes, so that childAlgorithm array can be updated */
    var compoundShapeRevision: Int

    init {
        val colObjWrap = if (isSwapped) body1Wrap else body0Wrap
        assert(colObjWrap.collisionShape.isCompound)

        val compoundShape = colObjWrap.collisionShape as CompoundShape
        compoundShapeRevision = compoundShape.updateRevision

        preallocateChildAlgorithms(body0Wrap, body1Wrap)
    }

    open fun  removeChildAlgorithms() {
        childCollisionAlgorithms.filterNotNull().forEach { dispatcher!!.freeCollisionAlgorithm(it) }
        childCollisionAlgorithms.clear()
    }

    fun preallocateChildAlgorithms(body0Wrap: CollisionObjectWrapper, body1Wrap: CollisionObjectWrapper) {
        val colObjWrap = if (isSwapped) body1Wrap else body0Wrap
        val otherObjWrap = if (isSwapped) body0Wrap else body1Wrap
        assert(colObjWrap.collisionShape.isCompound)

        val compoundShape = colObjWrap.collisionShape as CompoundShape
        val numChildren = compoundShape.numChildShapes

        childCollisionAlgorithms.resize(numChildren)
        for (i in 0 until numChildren)
            if (compoundShape.dynamicAabbTree != null) childCollisionAlgorithms[i] = null
            else {
                val childShape = compoundShape.getChildShape(i)
                //wrong child trans, but unused (hopefully)
                val childWrap = CollisionObjectWrapper(colObjWrap, childShape, colObjWrap.collisionObject!!, colObjWrap.worldTransform, -1, i)
                childCollisionAlgorithms[i] = dispatcher!!.findAlgorithm(childWrap, otherObjWrap, sharedManifold, DispatcherQueryType.CONTACT_POINT_ALGORITHMS)
            }
    }

    fun getChildAlgorithm(n: Int) = childCollisionAlgorithms[n]

    override fun processCollision(body0Wrap: CollisionObjectWrapper, body1Wrap: CollisionObjectWrapper, dispatchInfo: DispatcherInfo, resultOut: ManifoldResult) {

        val colObjWrap = if (isSwapped) body1Wrap else body0Wrap
        val otherObjWrap = if (isSwapped) body0Wrap else body1Wrap

        assert(colObjWrap.collisionShape.isCompound)
        val compoundShape = colObjWrap.collisionShape as CompoundShape

        // CompoundShape might have changed: make sure the internal child collision algorithm caches are still valid
        if (compoundShape.updateRevision != compoundShapeRevision) {
            // clear and update all
            removeChildAlgorithms()
            preallocateChildAlgorithms(body0Wrap, body1Wrap)
            compoundShapeRevision = compoundShape.updateRevision
        }

        if (childCollisionAlgorithms.isEmpty()) return

        val tree = compoundShape.dynamicAabbTree
        // use a dynamic aabb tree to cull potential child-overlaps
        val callback = CompoundLeafCallback(colObjWrap, otherObjWrap, dispatcher!!, dispatchInfo, resultOut, childCollisionAlgorithms, sharedManifold!!)

        /*  we need to refresh all contact manifolds
            note that we should actually recursively traverse all children, CompoundShape can nested more then 1 level
            deep so we should add a 'refreshManifolds' in the btCollisionAlgorithm */
        manifoldArray.clear()
        childCollisionAlgorithms.filterNotNull().forEach { al ->
            al.getAllContactManifolds(manifoldArray)
            manifoldArray.filter { it.numContacts != 0 }.forEach { mn ->
                with(resultOut) {
                    manifold = mn
                    refreshContactPoints()
                    manifold = null //??necessary?
                }
            }
            manifoldArray.clear()
        }

        if (tree != null) {
            val localAabbMin = Vec3()
            val localAabbMax = Vec3()
            val otherInCompoundSpace = colObjWrap.worldTransform.inverse() * otherObjWrap.worldTransform
            otherObjWrap.collisionShape.getAabb(otherInCompoundSpace, localAabbMin, localAabbMax)
            val extraExtends = Vec3(resultOut.closestPointDistanceThreshold)
            localAabbMin -= extraExtends
            localAabbMax += extraExtends

            val bounds = DbvtVolume.fromMM(localAabbMin, localAabbMax)
            // process all children, that overlap with  the given AABB bounds
            tree.collideTVNoStackAlloc(tree.root, bounds, stack2, callback)
        } else {
            // iterate over all children, perform an AABB check inside ProcessChildShape
            val numChildren = childCollisionAlgorithms.size
            for (i in 0 until numChildren)
                callback.processChildShape(compoundShape.getChildShape(i), i)
        }

        //iterate over all children, perform an AABB check inside ProcessChildShape
        val numChildren = childCollisionAlgorithms.size
        manifoldArray.clear()

        val aabbMin0 = Vec3()
        val aabbMax0 = Vec3()
        val aabbMin1 = Vec3()
        val aabbMax1 = Vec3()

        for (i in 0 until numChildren)
            if (childCollisionAlgorithms[i] != null) {
                val childShape = compoundShape.getChildShape(i)
                //if not longer overlapping, remove the algorithm
                val orgTrans = colObjWrap.worldTransform

                val childTrans = compoundShape.getChildTransform(i)
                val newChildWorldTrans = orgTrans * childTrans

                //perform an AABB check first
                childShape.getAabb(newChildWorldTrans, aabbMin0, aabbMax0)
                otherObjWrap.collisionShape.getAabb(otherObjWrap.worldTransform, aabbMin1, aabbMax1)

                if (!testAabbAgainstAabb2(aabbMin0, aabbMax0, aabbMin1, aabbMax1)) {
                    dispatcher!!.freeCollisionAlgorithm(childCollisionAlgorithms[i])
                    childCollisionAlgorithms[i] = null
                }
            }
    }

    override fun calculateTimeOfImpact(body0: CollisionObject, body1: CollisionObject, dispatchInfo: DispatcherInfo,
                                       resultOut: ManifoldResult): Float {
        assert(false)
        // needs to be fixed, using btCollisionObjectWrapper and NOT modifying internal data structures
        val colObj = if (isSwapped) body1 else body0
        val otherObj = if (isSwapped) body0 else body1

        assert(colObj.collisionShape!!.isCompound)

        val compoundShape = colObj.collisionShape as CompoundShape

        /*  We will use the OptimizedBVH, AABB tree to cull potential child-overlaps
            If both proxies are Compound, we will deal with that directly, by performing sequential/parallel tree
            traversals
            given Proxy0 and Proxy1, if both have a tree, Tree0 and Tree1, this means:
            determine overlapping nodes of Proxy1 using Proxy0 AABB against Tree1
            then use each overlapping node AABB against Tree0 and vise versa. */
        var hitFraction = 1f

        val numChildren = childCollisionAlgorithms.size
        for (i in 0 until numChildren) {
            //btCollisionShape* childShape = compoundShape->getChildShape(i);
            //backup
            val orgTrans = colObj.getWorldTransform()

            val childTrans = compoundShape.getChildTransform(i)
            //btTransform	newChildWorldTrans = orgTrans*childTrans ;
            colObj.setWorldTransform(orgTrans * childTrans)

            //btCollisionShape* tmpShape = colObj->getCollisionShape();
            //colObj->internalSetTemporaryCollisionShape( childShape );
            val frac = childCollisionAlgorithms[i]!!.calculateTimeOfImpact(colObj, otherObj, dispatchInfo, resultOut)
            if (frac < hitFraction) hitFraction = frac
            //revert back
            //colObj->internalSetTemporaryCollisionShape( tmpShape);
            colObj.setWorldTransform(orgTrans)
        }
        return hitFraction
    }

    override fun getAllContactManifolds(manifoldArray: ArrayList<PersistentManifold>) {
        childCollisionAlgorithms.filterNotNull().forEach { it.getAllContactManifolds(manifoldArray) }
    }

    class CreateFunc : CollisionAlgorithmCreateFunc() {
        override fun createCollisionAlgorithm(info: CollisionAlgorithmConstructionInfo, body0Wrap: CollisionObjectWrapper,
                                              body1Wrap: CollisionObjectWrapper) =
                CompoundCollisionAlgorithm(info, body0Wrap, body1Wrap, false)
    }

    class SwappedCreateFunc : CollisionAlgorithmCreateFunc() {
        override fun createCollisionAlgorithm(info: CollisionAlgorithmConstructionInfo, body0Wrap: CollisionObjectWrapper,
                                              body1Wrap: CollisionObjectWrapper) =
                CompoundCollisionAlgorithm(info, body0Wrap, body1Wrap, true)
    }
}

class CompoundLeafCallback(val compoundObjWrap: CollisionObjectWrapper, val otherObjWrap: CollisionObjectWrapper,
                           val dispatcher: Dispatcher, val dispatchInfo: DispatcherInfo, val resultOut: ManifoldResult,
                           val childCollisionAlgorithms: ArrayList<CollisionAlgorithm?>, val sharedManifold: PersistentManifold) :
        Dbvt.Collide {

    fun processChildShape(childShape: CollisionShape, index: Int) {
        assert(index >= 0)
        val compoundShape = compoundObjWrap.collisionShape as CompoundShape
        assert(index < compoundShape.numChildShapes)
        //backup
        val orgTrans = Transform(compoundObjWrap.worldTransform)

        val childTrans = compoundShape.getChildTransform(index)
        val newChildWorldTrans = orgTrans * childTrans
        //perform an AABB check first
        val aabbMin0 = Vec3()
        val aabbMax0 = Vec3()
        childShape.getAabb(newChildWorldTrans, aabbMin0, aabbMax0)

        val extendAabb = Vec3(resultOut.closestPointDistanceThreshold)
        aabbMin0 -= extendAabb
        aabbMax0 += extendAabb

        val aabbMin1 = Vec3()
        val aabbMax1 = Vec3()
        otherObjWrap.collisionShape.getAabb(otherObjWrap.worldTransform, aabbMin1, aabbMax1)

        gCompoundChildShapePairCallback?.let { if (!it(otherObjWrap.collisionShape, childShape)) return }

        if (testAabbAgainstAabb2(aabbMin0, aabbMax0, aabbMin1, aabbMax1)) {

            val compoundWrap = CollisionObjectWrapper(compoundObjWrap, childShape, compoundObjWrap.collisionObject!!, newChildWorldTrans, -1, index)
            val algo = when {
                resultOut.closestPointDistanceThreshold > 0 -> dispatcher.findAlgorithm(compoundWrap, otherObjWrap, null, DispatcherQueryType.CLOSEST_POINT_ALGORITHMS)
                else -> { //the contactpoint is still projected back using the original inverted worldtrans
                    if (childCollisionAlgorithms[index] == null)
                        childCollisionAlgorithms[index] = dispatcher.findAlgorithm(compoundWrap, otherObjWrap, sharedManifold, DispatcherQueryType.CONTACT_POINT_ALGORITHMS)
                    childCollisionAlgorithms[index]
                }
            }

            val tmpWrap: CollisionObjectWrapper?
            // detect swapping case
            if (resultOut.body0Internal === compoundObjWrap.collisionObject) {
                tmpWrap = resultOut.body0Wrap
                resultOut.body0Wrap = compoundWrap
                resultOut.setShapeIdentifiersA(-1, index)
            } else {
                tmpWrap = resultOut.body1Wrap
                resultOut.body1Wrap = compoundWrap
                resultOut.setShapeIdentifiersB(-1, index)
            }

            algo!!.processCollision(compoundWrap, otherObjWrap, dispatchInfo, resultOut)

            if (false) TODO() // dispatchInfo.debugDraw?.getDebugMode() & btIDebugDraw::DBG_DrawAabb))
//            {
//                btVector3 worldAabbMin, worldAabbMax
//                m_dispatchInfo.m_debugDraw->drawAabb(aabbMin0, aabbMax0, btVector3(1, 1, 1))
//                m_dispatchInfo.m_debugDraw->drawAabb(aabbMin1, aabbMax1, btVector3(1, 1, 1))
//            }

            if (resultOut.body0Internal === compoundObjWrap.collisionObject) resultOut.body0Wrap = tmpWrap
            else resultOut.body1Wrap = tmpWrap
        }
    }

    override fun process(node: DbvtNode) {
        val index = node.dataAsInt

        val compoundShape = compoundObjWrap.collisionShape as CompoundShape
        val childShape = compoundShape.getChildShape(index)

        if (false) TODO()
//        if (m_dispatchInfo.m_debugDraw && (m_dispatchInfo.m_debugDraw->getDebugMode() & btIDebugDraw::DBG_DrawAabb))
//        {
//            btVector3 worldAabbMin, worldAabbMax
//            btTransform orgTrans = m_compoundColObjWrap->getWorldTransform()
//            btTransformAabb(leaf->volume.Mins(), leaf->volume.Maxs(), 0., orgTrans, worldAabbMin, worldAabbMax)
//            m_dispatchInfo.m_debugDraw->drawAabb(worldAabbMin, worldAabbMax, btVector3(1, 0, 0))
//        }
//        #endif
        processChildShape(childShape, index)
    }
}
