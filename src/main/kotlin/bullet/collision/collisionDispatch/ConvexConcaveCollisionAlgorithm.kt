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

import bullet.collision.broadphaseCollision.CollisionAlgorithmConstructionInfo
import bullet.collision.broadphaseCollision.Dispatcher
import bullet.collision.broadphaseCollision.DispatcherInfo
import bullet.collision.broadphaseCollision.DispatcherQueryType
import bullet.collision.collisionShapes.TriangleCallback
import bullet.collision.collisionShapes.TriangleShape
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
    val manifoldPtr = dispatcher.getNewManifold(convexBodyWrap!!.collisionObject!!, triBodyWrap!!.collisionObject!!)

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
//        BT_PROFILE("btConvexTriangleCallback::processTriangle");

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

            val tm = TriangleShape(triangle[0], triangle[1], triangle[2]).apply { margin = collisionMarginTriangle }

            val triObWrap = CollisionObjectWrapper(triBodyWrap, tm, triBodyWrap!!.collisionObject!!, triBodyWrap!!.worldTransform,
                    partId, triangleIndex) //correct transform?
            val colAlgo = if (resultOut.closestPointDistanceThreshold > 0)
                ci.dispatcher!!.findAlgorithm(convexBodyWrap!!, triObWrap, null, DispatcherQueryType.CLOSEST_POINT_ALGORITHMS)
            else
                ci.dispatcher!!.findAlgorithm(convexBodyWrap!!, triObWrap, manifoldPtr, DispatcherQueryType.CONTACT_POINT_ALGORITHMS)

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

    fun clearCache() = dispatcher.clearManifold(manifoldPtr)
}

/** ConvexConcaveCollisionAlgorithm  supports collision between convex shapes and (concave) trianges meshes. */
class ConvexConcaveCollisionAlgorithm(ci: CollisionAlgorithmConstructionInfo, body0Wrap: CollisionObjectWrapper,
                                      body1Wrap: CollisionObjectWrapper, val isSwapped: Boolean) :
        ActivatingCollisionAlgorithm(ci, body0Wrap, body1Wrap) {

    val convexTriangleCallback = ConvexTriangleCallback(ci.dispatcher!!, body0Wrap, body1Wrap, isSwapped)


    public :

    BT_DECLARE_ALIGNED_ALLOCATOR()

    btConvexConcaveCollisionAlgorithm()

    virtual ~btConvexConcaveCollisionAlgorithm()

    virtual void processCollision(const btCollisionObjectWrapper * body0Wrap, const btCollisionObjectWrapper * body1Wrap, const btDispatcherInfo & dispatchInfo, btManifoldResult * resultOut)

    btScalar calculateTimeOfImpact (btCollisionObject * body0, btCollisionObject* body1, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)

    virtual void getAllContactManifolds(btManifoldArray& manifoldArray)

    void clearCache ()

    struct CreateFunc : public btCollisionAlgorithmCreateFunc
    {
        virtual btCollisionAlgorithm * CreateCollisionAlgorithm (btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap)
        {
            void * mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btConvexConcaveCollisionAlgorithm))
            return new(mem) btConvexConcaveCollisionAlgorithm (ci, body0Wrap, body1Wrap, false)
        }
    }

    struct SwappedCreateFunc : public btCollisionAlgorithmCreateFunc
    {
        virtual btCollisionAlgorithm * CreateCollisionAlgorithm (btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap)
        {
            void * mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btConvexConcaveCollisionAlgorithm))
            return new(mem) btConvexConcaveCollisionAlgorithm (ci, body0Wrap, body1Wrap, true)
        }
    }

}

#endif //BT_CONVEX_CONCAVE_COLLISION_ALGORITHM_H
