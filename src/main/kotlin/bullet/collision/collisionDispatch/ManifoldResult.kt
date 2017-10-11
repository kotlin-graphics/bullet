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

import bullet.collision.narrowPhaseCollision.DiscreteCollisionDetectorInterface
import bullet.collision.narrowPhaseCollision.ManifoldPoint
import bullet.collision.narrowPhaseCollision.PersistentManifold
import bullet.has
import bullet.linearMath.Vec3
import bullet.linearMath.planeSpace1
import bullet.collision.collisionDispatch.CollisionObject.CollisionFlags as CF
import bullet.collision.narrowPhaseCollision.ContactPointFlags as CPF

/** ManifoldResult is a helper class to manage  contact results.    */
class ManifoldResult : DiscreteCollisionDetectorInterface.Result {

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
            localA = body1Wrap.collisionObject!!.worldTransform.invXform(pointA)
            localB = body0Wrap.collisionObject!!.worldTransform.invXform(pointInWorld)
        } else {
            localA = body0Wrap.collisionObject!!.worldTransform.invXform(pointA)
            localB = body1Wrap.collisionObject!!.worldTransform.invXform(pointInWorld)
        }

        val newPt = ManifoldPoint(localA, localB, normalOnBInWorld, depth)
        newPt.positionWorldOnA(pointA)
        newPt.positionWorldOnB(pointInWorld)

        var insertIndex = manifoldPtr.getCacheEntry(newPt)

        newPt.combinedFriction = calculateCombinedFriction(body0Wrap.collisionObject, body1Wrap.collisionObject)
        newPt.combinedRestitution = calculateCombinedRestitution(body0Wrap.collisionObject, body1Wrap.collisionObject)
        newPt.combinedRollingFriction = calculateCombinedRollingFriction(body0Wrap.collisionObject, body1Wrap.collisionObject)
        newPt.combinedSpinningFriction = calculateCombinedSpinningFriction(body0Wrap.collisionObject, body1Wrap.collisionObject)

        if (body0Wrap.collisionObject!!.collisionFlags has CF.HAS_CONTACT_STIFFNESS_DAMPING.i ||
                body1Wrap.collisionObject!!.collisionFlags has CF.HAS_CONTACT_STIFFNESS_DAMPING.i) {
            newPt.combinedContactDamping1 = calculateCombinedContactDamping(body0Wrap.collisionObject, body1Wrap.collisionObject)
            newPt.combinedContactStiffness1 = calculateCombinedContactStiffness(body0Wrap.collisionObject, body1Wrap.collisionObject)
            newPt.contactPointFlags = newPt.contactPointFlags or CPF.CONTACT_STIFFNESS_DAMPING.i
        }
        if (body0Wrap.collisionObject!!.collisionFlags has CF.HAS_FRICTION_ANCHOR.i ||
                body1Wrap.collisionObject!!.collisionFlags has CF.HAS_FRICTION_ANCHOR.i)
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
        if (contactAddedCallback &&
                //and if either of the two bodies requires custom material
                ((m_body0Wrap->getCollisionObject()->getCollisionFlags() & btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK) ||
        (m_body1Wrap->getCollisionObject()->getCollisionFlags() & btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK)))
        {
            //experimental feature info, for per-triangle material etc.
            const btCollisionObjectWrapper * obj0Wrap = isSwapped ? m_body1Wrap : m_body0Wrap
                    const btCollisionObjectWrapper * obj1Wrap = isSwapped ? m_body0Wrap : m_body1Wrap
                    ( * gContactAddedCallback)(m_manifoldPtr->getContactPoint(insertIndex), obj0Wrap, newPt.m_partId0, newPt.m_index0, obj1Wrap, newPt.m_partId1, newPt.m_index1)
        }

        if (gContactStartedCallback && isNewCollision) {
            gContactStartedCallback(m_manifoldPtr)
        }
    }

    SIMD_FORCE_INLINE    void refreshContactPoints()
    {
        btAssert(m_manifoldPtr)
        if (!m_manifoldPtr->getNumContacts())
        return

        bool isSwapped = m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject()

        if (isSwapped)
            { m_manifoldPtr ->
                refreshContactPoints(m_body1Wrap->getCollisionObject()->getWorldTransform(), m_body0Wrap->getCollisionObject()->getWorldTransform())
            } else
            { m_manifoldPtr ->
                refreshContactPoints(m_body0Wrap->getCollisionObject()->getWorldTransform(), m_body1Wrap->getCollisionObject()->getWorldTransform())
            }
    }

    const btCollisionObjectWrapper* getBody0Wrap()
    const
    {
        return m_body0Wrap
    }
    const btCollisionObjectWrapper* getBody1Wrap()
    const
    {
        return m_body1Wrap
    }

    void setBody0Wrap(const btCollisionObjectWrapper* obj0Wrap)
    {
        m_body0Wrap = obj0Wrap
    }

    void setBody1Wrap(const btCollisionObjectWrapper* obj1Wrap)
    {
        m_body1Wrap = obj1Wrap
    }

    const btCollisionObject* getBody0Internal()
    const
    {
        return m_body0Wrap->getCollisionObject()
    }

    const btCollisionObject* getBody1Internal()
    const
    {
        return m_body1Wrap->getCollisionObject()
    }


    /// in the future we can let the user override the methods to combine restitution and friction
    static btScalar    calculateCombinedRestitution(const btCollisionObject* body0,const btCollisionObject* body1)
    static btScalar    calculateCombinedFriction(const btCollisionObject* body0,const btCollisionObject* body1)
    static btScalar calculateCombinedRollingFriction(const btCollisionObject* body0,const btCollisionObject* body1)
    static btScalar calculateCombinedSpinningFriction(const btCollisionObject* body0,const btCollisionObject* body1)
    static btScalar calculateCombinedContactDamping(const btCollisionObject* body0,const btCollisionObject* body1)
    static btScalar calculateCombinedContactStiffness(const btCollisionObject* body0,const btCollisionObject* body1)
}