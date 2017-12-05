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

import bullet.collision.broadphaseCollision.CollisionAlgorithm
import bullet.collision.broadphaseCollision.CollisionAlgorithmConstructionInfo
import bullet.collision.broadphaseCollision.DispatcherInfo
import bullet.collision.collisionShapes.ConvexShape
import bullet.collision.collisionShapes.StaticPlaneShape
import bullet.collision.narrowPhaseCollision.PersistentManifold
import bullet.collision.narrowPhaseCollision.gContactBreakingThreshold
import bullet.f
import bullet.linearMath.*

/** SphereBoxCollisionAlgorithm  provides sphere-box collision detection.
 *  Other features are frame-coherency (persistent data) and collision response. */
class ConvexPlaneCollisionAlgorithm(var manifold: PersistentManifold?, ci: CollisionAlgorithmConstructionInfo,
                                    body0Wrap: CollisionObjectWrapper, body1Wrap: CollisionObjectWrapper, val isSwapped: Boolean,
                                    val numPerturbationIterations: Int, val minimumPointsPerturbationThreshold: Int) :
        CollisionAlgorithm(ci) {

    var ownManifold = false

    init {
        val convexObjWrap = if (isSwapped) body1Wrap else body0Wrap
        val planeObjWrap = if (isSwapped) body0Wrap else body1Wrap

        if (manifold == null && dispatcher!!.needsCollision(convexObjWrap.collisionObject, planeObjWrap.collisionObject)) {
            manifold = dispatcher!!.getNewManifold(convexObjWrap.collisionObject!!, planeObjWrap.collisionObject!!)
            ownManifold = true
        }
    }

    override fun processCollision(body0Wrap: CollisionObjectWrapper, body1Wrap: CollisionObjectWrapper, dispatchInfo: DispatcherInfo,
                                  resultOut: ManifoldResult) {

        if (manifold == null) return

        val convexObjWrap = if (isSwapped) body1Wrap else body0Wrap
        val planeObjWrap = if (isSwapped) body0Wrap else body1Wrap

        val convexShape = convexObjWrap.collisionShape as ConvexShape
        val planeShape = planeObjWrap.collisionShape as StaticPlaneShape

        var hasCollision = false
        val planeNormal = planeShape.planeNormal
        val planeConstant = planeShape.planeConstant
        val planeInConvex = convexObjWrap.worldTransform.inverse() * planeObjWrap.worldTransform
        val convexInPlaneTrans = planeObjWrap.worldTransform.inverse() * convexObjWrap.worldTransform

        val vtx = convexShape.localGetSupportingVertex(planeInConvex.basis * -planeNormal)
        val vtxInPlane = convexInPlaneTrans(vtx)
        val distance = planeNormal.dot(vtxInPlane) - planeConstant

        val vtxInPlaneProjected = vtxInPlane - distance * planeNormal
        val vtxInPlaneWorld = planeObjWrap.worldTransform * vtxInPlaneProjected

        hasCollision = distance < manifold!!.contactBreakingThreshold
        resultOut.manifold = manifold
        if (hasCollision) { // report a contact. internally this will be kept persistent, and contact reduction is done
            val normalOnSurfaceB = planeObjWrap.worldTransform.basis * planeNormal
            val pOnB = Vec3(vtxInPlaneWorld)
            resultOut.addContactPoint(normalOnSurfaceB, pOnB, distance)
        }
        /*  the perturbation algorithm doesn't work well with implicit surfaces such as spheres, cylinder and cones:
            they keep on rolling forever because of the additional off-center contact points
            so only enable the feature for polyhedral shapes (BoxShape, ConvexHullShape etc) */
        if (convexShape.isPolyhedral && resultOut.manifold!!.numContacts < minimumPointsPerturbationThreshold) {
            val v0 = Vec3()
            val v1 = Vec3()
            planeSpace1(planeNormal, v0, v1)
            // now perform 'm_numPerturbationIterations' collision queries with the perturbated collision objects
            val angleLimit = 0.125f * PI
            val radius = convexShape.angularMotionDisc
            var perturbeAngle = gContactBreakingThreshold / radius
            if (perturbeAngle > angleLimit) perturbeAngle = angleLimit

            val perturbeRot = Quat(v0, perturbeAngle)
            for (i in 0 until numPerturbationIterations) {
                val iterationAngle = i * (PI2 / numPerturbationIterations.f)
                val rotQ = Quat(planeNormal, iterationAngle)
                collideSingleContact(rotQ.inverse() * perturbeRot * rotQ, body0Wrap, body1Wrap, dispatchInfo, resultOut)
            }
        }
        if (ownManifold && manifold!!.numContacts != 0) resultOut.refreshContactPoints()
    }

    fun collideSingleContact(perturbeRot: Quat, body0Wrap: CollisionObjectWrapper, body1Wrap: CollisionObjectWrapper,
                             dispatchInfo: DispatcherInfo, resultOut: ManifoldResult) {

        val convexObjWrap = if (isSwapped) body1Wrap else body0Wrap
        val planeObjWrap = if (isSwapped) body0Wrap else body1Wrap

        val convexShape = convexObjWrap.collisionShape as ConvexShape
        val planeShape = planeObjWrap.collisionShape as StaticPlaneShape

        var hasCollision = false
        val planeNormal = planeShape.planeNormal
        val planeConstant = planeShape.planeConstant

        val convexWorldTransform = Transform(convexObjWrap.worldTransform) // TODO check
        val convexInPlaneTrans = planeObjWrap.worldTransform.inverse() * convexWorldTransform
        //now perturbe the convex-world transform
        convexWorldTransform.basis *= Mat3(perturbeRot)
        val planeInConvex = convexWorldTransform.inverse() * planeObjWrap.worldTransform

        val vtx = convexShape.localGetSupportingVertex(planeInConvex.basis * -planeNormal)

        val vtxInPlane = convexInPlaneTrans(vtx)
        val distance = planeNormal.dot(vtxInPlane) - planeConstant

        val vtxInPlaneProjected = vtxInPlane - distance * planeNormal
        val vtxInPlaneWorld = planeObjWrap.worldTransform * vtxInPlaneProjected

        hasCollision = distance < manifold!!.contactBreakingThreshold
        resultOut.manifold = manifold
        if (hasCollision) { // report a contact. internally this will be kept persistent, and contact reduction is done
            val normalOnSurfaceB = planeObjWrap.worldTransform.basis * planeNormal
            val pOnB = Vec3(vtxInPlaneWorld)
            resultOut.addContactPoint(normalOnSurfaceB, pOnB, distance)
        }
    }

    override fun calculateTimeOfImpact(body0: CollisionObject, body1: CollisionObject, dispatchInfo: DispatcherInfo,
                                       resultOut: ManifoldResult) = 1f // not yet

    override fun getAllContactManifolds(manifoldArray: ArrayList<PersistentManifold>) {
        manifold?.let { if (ownManifold) manifoldArray.add(it) }
    }

    class CreateFunc : CollisionAlgorithmCreateFunc() {

        var numPerturbationIterations = 1
        var minimumPointsPerturbationThreshold = 0

        override fun createCollisionAlgorithm(info: CollisionAlgorithmConstructionInfo, body0Wrap: CollisionObjectWrapper,
                                              body1Wrap: CollisionObjectWrapper) =
                ConvexPlaneCollisionAlgorithm(null, info, body0Wrap, body1Wrap, swapped, numPerturbationIterations,
                        minimumPointsPerturbationThreshold)
    }
}