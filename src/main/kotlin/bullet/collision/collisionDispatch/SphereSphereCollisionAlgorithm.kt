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

import bullet.CLEAR_MANIFOLD
import bullet.EPSILON
import bullet.collision.broadphaseCollision.CollisionAlgorithmConstructionInfo
import bullet.collision.broadphaseCollision.DispatcherInfo
import bullet.collision.collisionShapes.SphereShape
import bullet.collision.narrowPhaseCollision.PersistentManifold
import bullet.linearMath.Vec3
import bullet.linearMath.times

/** SphereSphereCollisionAlgorithm  provides sphere-sphere collision detection.
 *  Other features are frame-coherency (persistent data) and collision response.
 *  Also provides the most basic sample for custom/user CollisionAlgorithm */
class SphereSphereCollisionAlgorithm : ActivatingCollisionAlgorithm {

    var ownManifold = false
    var manifold: PersistentManifold? = null

    constructor(mf: PersistentManifold?, ci: CollisionAlgorithmConstructionInfo, col0Wrap: CollisionObjectWrapper,
                col1Wrap: CollisionObjectWrapper) : super(ci, col0Wrap, col1Wrap) {
        manifold = mf ?: dispatcher!!.getNewManifold(col0Wrap.collisionObject!!, col1Wrap.collisionObject!!).also { ownManifold = true }
    }

    constructor(ci: CollisionAlgorithmConstructionInfo) : super(ci)

    override fun processCollision(body0Wrap: CollisionObjectWrapper, body1Wrap: CollisionObjectWrapper, dispatchInfo: DispatcherInfo, resultOut: ManifoldResult) {

        if (manifold == null) return

        resultOut.manifold = manifold

        val sphere0 = body0Wrap.collisionShape as SphereShape
        val sphere1 = body1Wrap.collisionShape as SphereShape

        val diff = body0Wrap.worldTransform.origin - body1Wrap.worldTransform.origin
        val len = diff.length()
        val radius0 = sphere0.radius
        val radius1 = sphere1.radius

        if (CLEAR_MANIFOLD) manifold!!.clearManifold() //don't do this, it disables warmstarting

        // iff distance positive, don't generate a new contact
        if (len > (radius0 + radius1 + resultOut.closestPointDistanceThreshold)) {
            if (!CLEAR_MANIFOLD) resultOut.refreshContactPoints()
            return
        }
        // distance (negative means penetration)
        val dist = len - (radius0 + radius1)

        val normalOnSurfaceB = if (len > Float.EPSILON) diff / len else Vec3 (1, 0, 0)

        // point on A (worldspace)
        // btVector3 pos0 = col0->getWorldTransform().getOrigin() - radius0 * normalOnSurfaceB;
        // point on B (worldspace)
        val pos1 = body1Wrap.worldTransform.origin+radius1* normalOnSurfaceB

        // report a contact. internally this will be kept persistent, and contact reduction is done

        resultOut.addContactPoint(normalOnSurfaceB, pos1, dist)

        if(!CLEAR_MANIFOLD) resultOut.refreshContactPoints()
    }

    override fun calculateTimeOfImpact(body0: CollisionObject, body1: CollisionObject, dispatchInfo: DispatcherInfo,
                                       resultOut: ManifoldResult) = 1f //not yet

    override fun getAllContactManifolds(manifoldArray: ArrayList<PersistentManifold>) {
        manifold?.let { if (ownManifold) manifoldArray.add(manifold!!) }
    }

    class CreateFunc : CollisionAlgorithmCreateFunc()    {
        override fun createCollisionAlgorithm(info: CollisionAlgorithmConstructionInfo, body0Wrap: CollisionObjectWrapper,
                                              body1Wrap: CollisionObjectWrapper) =
                SphereSphereCollisionAlgorithm (null, info, body0Wrap, body1Wrap)
    }
}
