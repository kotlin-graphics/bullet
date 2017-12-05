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

import bullet.USE_PERSISTENT_CONTACTS
import bullet.collision.broadphaseCollision.CollisionAlgorithmConstructionInfo
import bullet.collision.broadphaseCollision.DispatcherInfo
import bullet.collision.collisionShapes.BoxShape
import bullet.collision.narrowPhaseCollision.DiscreteCollisionDetectorInterface
import bullet.collision.narrowPhaseCollision.PersistentManifold
import bullet.linearMath.LARGE_FLOAT

/** box-box collision detection */
class BoxBoxCollisionAlgorithm : ActivatingCollisionAlgorithm {

    var ownManifold = false
    var manifold: PersistentManifold? = null

    constructor(ci: CollisionAlgorithmConstructionInfo) : super(ci)
    constructor(mf: PersistentManifold?, ci: CollisionAlgorithmConstructionInfo, body0Wrap: CollisionObjectWrapper,
                body1Wrap: CollisionObjectWrapper) : super(ci, body0Wrap, body1Wrap) {
        manifold = mf
        if (manifold == null && dispatcher!!.needsCollision(body0Wrap.collisionObject, body1Wrap.collisionObject)) {
            manifold = dispatcher!!.getNewManifold(body0Wrap.collisionObject!!, body1Wrap.collisionObject!!)
            ownManifold = true
        }
    }

    override fun processCollision(body0Wrap: CollisionObjectWrapper, body1Wrap: CollisionObjectWrapper, dispatchInfo: DispatcherInfo, resultOut: ManifoldResult) {

        if (manifold == null) return

        val box0 = body0Wrap.collisionShape as BoxShape
        val box1 = body1Wrap.collisionShape as BoxShape

        // report a contact. internally this will be kept persistent, and contact reduction is done
        resultOut.manifold = manifold
        if (!USE_PERSISTENT_CONTACTS) manifold!!.clearManifold()

        val input = DiscreteCollisionDetectorInterface.ClosestPointInput().apply {
            maximumDistanceSquared = LARGE_FLOAT
            transformA put body0Wrap.worldTransform
            transformB put body1Wrap.worldTransform
        }

        val detector = BoxBoxDetector(box0, box1)
        detector.getClosestPoints(input, resultOut, dispatchInfo.debugDraw)

        //  refreshContactPoints is only necessary when using persistent contact points. otherwise all points are newly added
        if (USE_PERSISTENT_CONTACTS) if (ownManifold) resultOut.refreshContactPoints()
    }

    override fun calculateTimeOfImpact(body0: CollisionObject, body1: CollisionObject, dispatchInfo: DispatcherInfo,
                                       resultOut: ManifoldResult) = 1f //not yet

    override fun getAllContactManifolds(manifoldArray: ArrayList<PersistentManifold>) {
        manifold?.let { if (ownManifold) manifoldArray.add(it) }
    }

    class CreateFunc : CollisionAlgorithmCreateFunc() {
        override fun createCollisionAlgorithm(info: CollisionAlgorithmConstructionInfo, body0Wrap: CollisionObjectWrapper,
                                              body1Wrap: CollisionObjectWrapper) =
                BoxBoxCollisionAlgorithm(null, info, body0Wrap, body1Wrap)
    }
}
