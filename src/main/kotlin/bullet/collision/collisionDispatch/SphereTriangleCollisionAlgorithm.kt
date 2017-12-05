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
import bullet.collision.broadphaseCollision.DispatcherInfo
import bullet.collision.collisionShapes.SphereShape
import bullet.collision.collisionShapes.TriangleShape
import bullet.collision.narrowPhaseCollision.DiscreteCollisionDetectorInterface
import bullet.collision.narrowPhaseCollision.PersistentManifold
import bullet.linearMath.LARGE_FLOAT

/** SphereSphereCollisionAlgorithm  provides sphere-sphere collision detection.
 *  Other features are frame-coherency (persistent data) and collision response.
 *  Also provides the most basic sample for custom/user CollisionAlgorithm */
class SphereTriangleCollisionAlgorithm : ActivatingCollisionAlgorithm {

    var ownManifold = false
    var manifold: PersistentManifold? = null
    var swapped = false

    constructor(mf: PersistentManifold?, ci: CollisionAlgorithmConstructionInfo, body0Wrap: CollisionObjectWrapper,
                body1Wrap: CollisionObjectWrapper, swapped: Boolean) : super(ci, body0Wrap, body1Wrap) {
        manifold = mf ?: dispatcher!!.getNewManifold(body0Wrap.collisionObject!!, body1Wrap.collisionObject!!).also { ownManifold = true }
        this.swapped = swapped
    }

    constructor(ci: CollisionAlgorithmConstructionInfo) : super(ci)

    override fun processCollision(body0Wrap: CollisionObjectWrapper, body1Wrap: CollisionObjectWrapper, dispatchInfo: DispatcherInfo, resultOut: ManifoldResult) {

        if (manifold == null) return

        val sphereObjWrap = if (swapped) body1Wrap else body0Wrap
        val triObjWrap = if (swapped) body0Wrap else body1Wrap

        val sphere = sphereObjWrap.collisionShape as SphereShape
        val triangle = triObjWrap.collisionShape as TriangleShape

        /// report a contact. internally this will be kept persistent, and contact reduction is done
        resultOut.manifold = manifold
        val detector = SphereTriangleDetector(sphere, triangle, manifold!!.contactBreakingThreshold +
                resultOut.closestPointDistanceThreshold)

        val input = DiscreteCollisionDetectorInterface.ClosestPointInput().apply {
            maximumDistanceSquared = LARGE_FLOAT ///@todo: tighter bounds
            transformA put sphereObjWrap.worldTransform
            transformB put triObjWrap.worldTransform
        }

        detector.getClosestPoints(input, resultOut, dispatchInfo.debugDraw, swapResults = swapped)
        if (ownManifold) resultOut.refreshContactPoints()
    }

    override fun calculateTimeOfImpact(body0: CollisionObject, body1: CollisionObject, dispatchInfo: DispatcherInfo,
                                       resultOut: ManifoldResult) = 1f //not yet

    override fun getAllContactManifolds(manifoldArray: ArrayList<PersistentManifold>) {
        manifold?.let { if (ownManifold) manifoldArray.add(it) }
    }

    class CreateFunc : CollisionAlgorithmCreateFunc() {

        override fun createCollisionAlgorithm(info: CollisionAlgorithmConstructionInfo, body0Wrap: CollisionObjectWrapper,
                                              body1Wrap: CollisionObjectWrapper) =
                SphereTriangleCollisionAlgorithm(info.manifold, info, body0Wrap, body1Wrap, swapped)
    }
}