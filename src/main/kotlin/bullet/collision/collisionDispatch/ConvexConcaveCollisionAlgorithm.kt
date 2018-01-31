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

import bullet.BT_PROFILE
import bullet.collision.broadphaseCollision.CollisionAlgorithmConstructionInfo
import bullet.collision.broadphaseCollision.Dispatcher
import bullet.collision.broadphaseCollision.DispatcherInfo
import bullet.collision.broadphaseCollision.DispatcherQueryType
import bullet.collision.collisionShapes.ConcaveShape
import bullet.collision.collisionShapes.SphereShape
import bullet.collision.collisionShapes.TriangleCallback
import bullet.collision.collisionShapes.TriangleShape
import bullet.collision.narrowPhaseCollision.ConvexCast
import bullet.collision.narrowPhaseCollision.PersistentManifold
import bullet.collision.narrowPhaseCollision.SubsimplexConvexCast
import bullet.collision.narrowPhaseCollision.VoronoiSimplexSolver
import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import bullet.linearMath.testTriangleAgainstAabb2

/** For each triangle in the concave mesh that overlaps with the AABB of a convex (convexProxy), processTriangle is
 *  called. */
class ConvexTriangleCallback(val dispatcher: Dispatcher, body0Wrap: CollisionObjectWrapper, body1Wrap: CollisionObjectWrapper,
                             isSwapped: Boolean) : TriangleCallback {

    val aabbMin = Vec3()
    val aabbMax = Vec3()

    var convexBodyWrap: CollisionObjectWrapper? = if (isSwapped) body1Wrap else body0Wrap
    var triBodyWrap: CollisionObjectWrapper? = if (isSwapped) body0Wrap else body1Wrap

    var resultOut = ManifoldResult()
    var dispatchInfoPtr: DispatcherInfo? = null
    var collisionMarginTriangle = 0f

    var triangleCount = 0
    // create the manifold from the dispatcher 'manifold pool'
    val manifold = dispatcher.getNewManifold(convexBodyWrap!!.collisionObject!!, triBodyWrap!!.collisionObject!!)

    fun setTimeStepAndCounters(collisionMarginTriangle: Float, dispatchInfo: DispatcherInfo, convexBodyWrap: CollisionObjectWrapper,
                               triBodyWrap: CollisionObjectWrapper, resultOut: ManifoldResult) {

        this.convexBodyWrap = convexBodyWrap
        this.triBodyWrap = triBodyWrap

        dispatchInfoPtr = dispatchInfo
        this.collisionMarginTriangle = collisionMarginTriangle
        this.resultOut = resultOut

        //recalc aabbs
        val convexInTriangleSpace = triBodyWrap.worldTransform.inverse() * convexBodyWrap.worldTransform
        val convexShape = convexBodyWrap.collisionShape
        //CollisionShape* triangleShape = static_cast<btCollisionShape*>(triBody->m_collisionShape);
        convexShape.getAabb(convexInTriangleSpace, aabbMin, aabbMax)
        val extraMargin = collisionMarginTriangle + resultOut.closestPointDistanceThreshold

        val extra = Vec3(extraMargin)

        aabbMax += extra
        aabbMin -= extra
    }

    fun clearWrapperData() {
        convexBodyWrap = null
        triBodyWrap = null
    }

    override fun processTriangle(triangle: Array<Vec3>, partId: Int, triangleIndex: Int) {
        BT_PROFILE("ConvexTriangleCallback::processTriangle")

        if (!testTriangleAgainstAabb2(triangle, aabbMin, aabbMax)) return

        //just for debugging purposes
        //printf("triangle %d",m_triangleCount++);

        val ci = CollisionAlgorithmConstructionInfo()
        ci.dispatcher = dispatcher

        // debug drawing of the overlapping triangles TODO
//        if(false && dispatchInfoPtr?.debugDraw?.getDebugMode() &btIDebugDraw::DBG_DrawWireframe ))
//        {
//            const btCollisionObject* ob = const_cast<btCollisionObject*>(m_triBodyWrap->getCollisionObject());
//            btVector3 color(1,1,0);
//            btTransform& tr = ob->getWorldTransform();
//            m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[0]),tr(triangle[1]),color);
//            m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[1]),tr(triangle[2]),color);
//            m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[2]),tr(triangle[0]),color);
//        }

        if (convexBodyWrap!!.collisionShape.isConvex) {

            val tm = TriangleShape(triangle).apply { margin = collisionMarginTriangle }

            val triObWrap = CollisionObjectWrapper(triBodyWrap, tm, triBodyWrap!!.collisionObject!!, triBodyWrap!!.worldTransform,
                    partId, triangleIndex) //correct transform?
            val colAlgo = if (resultOut.closestPointDistanceThreshold > 0)
                ci.dispatcher!!.findAlgorithm(convexBodyWrap!!, triObWrap, null, DispatcherQueryType.CLOSEST_POINT_ALGORITHMS)
            else
                ci.dispatcher!!.findAlgorithm(convexBodyWrap!!, triObWrap, manifold, DispatcherQueryType.CONTACT_POINT_ALGORITHMS)

            val tmpWrap: CollisionObjectWrapper?
            if (resultOut.body0Internal === triBodyWrap!!.collisionObject) {
                tmpWrap = resultOut.body0Wrap
                resultOut.body0Wrap = triObWrap
                resultOut.setShapeIdentifiersA(partId, triangleIndex)
            } else {
                tmpWrap = resultOut.body1Wrap
                resultOut.body1Wrap = triObWrap
                resultOut.setShapeIdentifiersB(partId, triangleIndex)
            }

            colAlgo!!.processCollision(convexBodyWrap!!, triObWrap, dispatchInfoPtr!!, resultOut)

            if (resultOut.body0Internal === triBodyWrap!!.collisionObject)
                resultOut.body0Wrap = tmpWrap
            else
                resultOut.body1Wrap = tmpWrap

            ci.dispatcher!!.freeCollisionAlgorithm(colAlgo)
        }
    }

    fun clearCache() = dispatcher.clearManifold(manifold)
}

/** ConvexConcaveCollisionAlgorithm  supports collision between convex shapes and (concave) trianges meshes. */
class ConvexConcaveCollisionAlgorithm(ci: CollisionAlgorithmConstructionInfo, body0Wrap: CollisionObjectWrapper,
                                      body1Wrap: CollisionObjectWrapper, val isSwapped: Boolean) :
        ActivatingCollisionAlgorithm(ci, body0Wrap, body1Wrap) {

    val convexTriangleCallback = ConvexTriangleCallback(ci.dispatcher!!, body0Wrap, body1Wrap, isSwapped)

    override fun processCollision(body0Wrap: CollisionObjectWrapper, body1Wrap: CollisionObjectWrapper, dispatchInfo: DispatcherInfo, resultOut: ManifoldResult) {
        BT_PROFILE("ConvexConcaveCollisionAlgorithm::processCollision")

        val convexBodyWrap = if (isSwapped) body1Wrap else body0Wrap
        val triBodyWrap = if (isSwapped) body0Wrap else body1Wrap

        if (triBodyWrap.collisionShape.isConcave) {
            val concaveShape = triBodyWrap.collisionShape as ConcaveShape

            if (convexBodyWrap.collisionShape.isConvex) {
                val collisionMarginTriangle = concaveShape.margin

                resultOut.manifold = convexTriangleCallback.manifold
                convexTriangleCallback.setTimeStepAndCounters(collisionMarginTriangle, dispatchInfo, convexBodyWrap, triBodyWrap, resultOut)

                convexTriangleCallback.manifold.setBodies(convexBodyWrap.collisionObject!!, triBodyWrap.collisionObject!!)

                concaveShape.processAllTriangles(convexTriangleCallback, convexTriangleCallback.aabbMin, convexTriangleCallback.aabbMax)

                resultOut.refreshContactPoints()

                convexTriangleCallback.clearWrapperData()
            }
        }
    }

    override fun calculateTimeOfImpact(body0: CollisionObject, body1: CollisionObject, dispatchInfo: DispatcherInfo, resultOut: ManifoldResult): Float {

        val convexbody = if (isSwapped) body1 else body0
        val triBody = if (isSwapped) body0 else body1

        //quick approximation using raycast, todo: hook up to the continuous collision detection (one of the btConvexCast)

        /*  only perform CCD above a certain threshold, this prevents blocking on the long run
            because object in a blocked ccd state (hitfraction<1) get their linear velocity halved each frame...         */
        val squareMot0 = (convexbody.getInterpolationWorldTransform().origin - convexbody.getWorldTransform().origin).length2()
        if (squareMot0 < convexbody.ccdSquareMotionThreshold) return 1f

        //const btVector3& from = convexbody->m_worldTransform.getOrigin();
        //btVector3 to = convexbody->m_interpolationWorldTransform.getOrigin();
        //todo: only do if the motion exceeds the 'radius'

        val triInv = triBody.getWorldTransform().inverse()
        val convexFromLocal = triInv * convexbody.getWorldTransform()
        val convexToLocal = triInv * convexbody.getInterpolationWorldTransform()

        class LocalTriangleSphereCastCallback(val ccdSphereFromTrans: Transform, val ccdSphereToTrans: Transform,
                                              val ccdSphereRadius: Float, var hitFraction: Float) : TriangleCallback {

            override fun processTriangle(triangle: Array<Vec3>, partId: Int, triangleIndex: Int) {
                BT_PROFILE("processTriangle")

                //do a swept sphere for now
                val ident = Transform().apply { setIdentity() }
                val castResult = ConvexCast.CastResult().apply { fraction = hitFraction }
                val pointShape = SphereShape(ccdSphereRadius)
                val triShape = TriangleShape(triangle)
                val simplexSolver = VoronoiSimplexSolver()
                val convexCaster = SubsimplexConvexCast(pointShape, triShape, simplexSolver)
                //GjkConvexCast	convexCaster(&pointShape,convexShape,&simplexSolver);
                //ContinuousConvexCollision convexCaster(&pointShape,convexShape,&simplexSolver,0);
                //local space?

                if (convexCaster.calcTimeOfImpact(ccdSphereFromTrans, ccdSphereToTrans, ident, ident, castResult))
                    if (hitFraction > castResult.fraction) hitFraction = castResult.fraction
            }
        }

        if (triBody.collisionShape!!.isConcave) {
            val rayAabbMin = convexFromLocal.origin min convexToLocal.origin
            val rayAabbMax = convexFromLocal.origin max convexToLocal.origin
            val ccdRadius0 = convexbody.ccdSweptSphereRadius
            rayAabbMin -= Vec3(ccdRadius0)
            rayAabbMax += Vec3(ccdRadius0)

            val curHitFraction = 1f //is this available?
            val raycastCallback = LocalTriangleSphereCastCallback(convexFromLocal, convexToLocal, convexbody.ccdSweptSphereRadius, curHitFraction)
                    .apply { hitFraction = convexbody.hitFraction }

            // triBody = concavebody
            (triBody.collisionShape as? ConcaveShape)?.processAllTriangles(raycastCallback, rayAabbMin, rayAabbMax)

            if (raycastCallback.hitFraction < convexbody.hitFraction) {
                convexbody.hitFraction = raycastCallback.hitFraction
                return raycastCallback.hitFraction
            }
        }
        return 1f
    }

    override fun getAllContactManifolds(manifoldArray: ArrayList<PersistentManifold>) {
        manifoldArray.add(convexTriangleCallback.manifold) // TODO cpp has nullability check on manifold
    }

    fun clearCache() {
        convexTriangleCallback.clearCache()
    }

    class CreateFunc : CollisionAlgorithmCreateFunc() {
        override fun createCollisionAlgorithm(info: CollisionAlgorithmConstructionInfo, body0Wrap: CollisionObjectWrapper,
                                              body1Wrap: CollisionObjectWrapper) =
                ConvexConcaveCollisionAlgorithm(info, body0Wrap, body1Wrap, false)
    }

    class SwappedCreateFunc : CollisionAlgorithmCreateFunc() {
        override fun createCollisionAlgorithm(info: CollisionAlgorithmConstructionInfo, body0Wrap: CollisionObjectWrapper,
                                              body1Wrap: CollisionObjectWrapper) =
                ConvexConcaveCollisionAlgorithm(info, body0Wrap, body1Wrap, true)
    }
}