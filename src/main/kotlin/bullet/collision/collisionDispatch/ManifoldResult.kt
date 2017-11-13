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

import bullet.collision.narrowPhaseCollision.DiscreteCollisionDetector
import bullet.collision.narrowPhaseCollision.ManifoldPoint
import bullet.collision.narrowPhaseCollision.PersistentManifold
import bullet.collision.narrowPhaseCollision.contactStartedCallback
import bullet.has
import bullet.linearMath.Vec3
import bullet.linearMath.planeSpace1
import bullet.collision.collisionDispatch.CollisionObject.CollisionFlags as CF
import bullet.collision.narrowPhaseCollision.ContactPointFlags as CPF

typealias ContactAddedCallback = (ManifoldPoint, CollisionObjectWrapper, Int, Int, CollisionObjectWrapper, Int, Int) -> Boolean
/** This is to allow MaterialCombiner/Custom Friction/Restitution values    */
var contactAddedCallback: ContactAddedCallback? = null

/** ManifoldResult is a helper class to manage  contact results.    */
open class ManifoldResult : DiscreteCollisionDetector.Result {

    var manifoldPtr: PersistentManifold? = null

    var body0Wrap: CollisionObjectWrapper? = null
    var body1Wrap: CollisionObjectWrapper? = null
    var partId0 = 0
    var partId1 = 0
    var index0 = 0
    var index1 = 0

    var closestPointDistanceThreshold = 0f

    constructor()
    constructor(body0Wrap: CollisionObjectWrapper, body1Wrap: CollisionObjectWrapper) {
        this.body0Wrap = body0Wrap
        this.body1Wrap = body1Wrap
    }

    override fun setShapeIdentifiersA(partId0: Int, index0: Int) {
        this.partId0 = partId0
        this.index0 = index0
    }

    override fun setShapeIdentifiersB(partId1: Int, index1: Int) {
        this.partId1 = partId1
        this.index1 = index1
    }

    override fun addContactPoint(normalOnBInWorld: Vec3, pointInWorld: Vec3, depth: Float) {
        val manifoldPtr = manifoldPtr!!
        val body0Wrap = body0Wrap!!
        val body1Wrap = body1Wrap!!
        val co0 = body0Wrap.collisionObject!!
        val co1 = body0Wrap.collisionObject!!
        //order in manifold needs to match

        if (depth > manifoldPtr.contactBreakingThreshold)
//	if (depth > m_manifoldPtr->getContactProcessingThreshold())
            return

        val isSwapped = manifoldPtr.body0 !== body0Wrap.collisionObject
        val isNewCollision = manifoldPtr.numContacts == 0

        val pointA = pointInWorld + normalOnBInWorld * depth

        val localA: Vec3
        val localB: Vec3

        if (isSwapped) {
            localA = co1.getWorldTransform().invXform(pointA)
            localB = co0.getWorldTransform().invXform(pointInWorld)
        } else {
            localA = co0.getWorldTransform().invXform(pointA)
            localB = co1.getWorldTransform().invXform(pointInWorld)
        }

        val newPt = ManifoldPoint(localA, localB, normalOnBInWorld, depth)
        newPt.positionWorldOnA(pointA)
        newPt.positionWorldOnB(pointInWorld)

        var insertIndex = manifoldPtr.getCacheEntry(newPt)

        newPt.combinedFriction = calculateCombinedFriction(co0, co1)
        newPt.combinedRestitution = calculateCombinedRestitution(co0, co1)
        newPt.combinedRollingFriction = calculateCombinedRollingFriction(co0, co1)
        newPt.combinedSpinningFriction = calculateCombinedSpinningFriction(co0, co1)

        if (co0.collisionFlags has CF.HAS_CONTACT_STIFFNESS_DAMPING.i || co1.collisionFlags has CF.HAS_CONTACT_STIFFNESS_DAMPING.i) {
            newPt.combinedContactDamping1 = calculateCombinedContactDamping(co0, co1)
            newPt.combinedContactStiffness1 = calculateCombinedContactStiffness(co0, co1)
            newPt.contactPointFlags = newPt.contactPointFlags or CPF.CONTACT_STIFFNESS_DAMPING.i
        }
        if (co0.collisionFlags has CF.HAS_FRICTION_ANCHOR.i || co1.collisionFlags has CF.HAS_FRICTION_ANCHOR.i)
            newPt.contactPointFlags = newPt.contactPointFlags or CPF.FRICTION_ANCHOR.i

        planeSpace1(newPt.normalWorldOnB, newPt.lateralFrictionDir1, newPt.lateralFrictionDir2)

        //BP mod, store contact triangles.
        if (isSwapped) {
            newPt.partId0 = partId1
            newPt.partId1 = partId0
            newPt.index0 = index1
            newPt.index1 = index0
        } else {
            newPt.partId0 = partId0
            newPt.partId1 = partId1
            newPt.index0 = index0
            newPt.index1 = index1
        }
        //printf("depth=%f\n",depth);
        ///@todo, check this for any side effects
        if (insertIndex >= 0)
        //const btManifoldPoint& oldPoint = m_manifoldPtr->getContactPoint(insertIndex);
            manifoldPtr.replaceContactPoint(newPt, insertIndex)
        else
            insertIndex = manifoldPtr.addManifoldPoint(newPt)

        //User can override friction and/or restitution
        contactAddedCallback?.let {
            //and if either of the two bodies requires custom material
            if (co0.collisionFlags has CF.CUSTOM_MATERIAL_CALLBACK.i || co1.collisionFlags has CF.CUSTOM_MATERIAL_CALLBACK.i) {
                //experimental feature info, for per-triangle material etc.
                val obj0Wrap = if (isSwapped) body1Wrap else body0Wrap
                val obj1Wrap = if (isSwapped) body0Wrap else body1Wrap
                it(manifoldPtr.getContactPoint(insertIndex), obj0Wrap, newPt.partId0, newPt.index0, obj1Wrap, newPt.partId1, newPt.index1)
            }
        }
        contactStartedCallback?.let { if (isNewCollision) it(manifoldPtr) }
    }

    fun refreshContactPoints() {
        val manifoldPtr = manifoldPtr!!
        if (manifoldPtr.numContacts == 0) return
        val co0 = body0Wrap!!.collisionObject!!
        val co1 = body1Wrap!!.collisionObject!!
        if (manifoldPtr.body0 !== body0Wrap!!.collisionObject)  // is swapped
            manifoldPtr.refreshContactPoints(co1.getWorldTransform(), co0.getWorldTransform())
        else
            manifoldPtr.refreshContactPoints(co0.getWorldTransform(), co1.getWorldTransform())
    }

    val body0Internal get() = body0Wrap!!.collisionObject
    val body1Internal get() = body1Wrap!!.collisionObject

    companion object {

        val MAX_FRICTION = 10f

        // in the future we can let the user override the methods to combine restitution and friction
        fun calculateCombinedRestitution(body0: CollisionObject, body1: CollisionObject) = body0.restitution * body1.restitution

        /** User can override this material combiner by implementing contactAddedCallback and setting
         *  body0.collisionFlags |= CollisionObject.customMaterialCallback  */
        fun calculateCombinedFriction(body0: CollisionObject, body1: CollisionObject): Float {
            val friction = body0.friction * body1.friction
            return when {
                friction < -MAX_FRICTION -> -MAX_FRICTION
                friction > MAX_FRICTION -> MAX_FRICTION
                else -> friction
            }
        }

        fun calculateCombinedRollingFriction(body0: CollisionObject, body1: CollisionObject): Float {
            val friction = body0.rollingFriction * body1.friction + body1.rollingFriction * body0.friction
            return when {
                friction < -MAX_FRICTION -> -MAX_FRICTION
                friction > MAX_FRICTION -> MAX_FRICTION
                else -> friction
            }
        }

        fun calculateCombinedSpinningFriction(body0: CollisionObject, body1: CollisionObject): Float {
            val friction = body0.spinningFriction * body1.friction + body1.spinningFriction * body0.friction
            return when {
                friction < -MAX_FRICTION -> -MAX_FRICTION
                friction > MAX_FRICTION -> MAX_FRICTION
                else -> friction
            }
        }

        fun calculateCombinedContactDamping(body0: CollisionObject, body1: CollisionObject) = body0.contactDamping + body1.contactDamping
        fun calculateCombinedContactStiffness(body0: CollisionObject, body1: CollisionObject) = 1f / (1f / body0.contactStiffness + 1f / body1.contactStiffness)
    }
}