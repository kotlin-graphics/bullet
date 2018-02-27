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

import bullet.EPSILON
import bullet.collision.broadphaseCollision.BroadphaseProxy
import bullet.collision.collisionShapes.CollisionShape
import bullet.has
import bullet.hasnt
import bullet.linearMath.LARGE_FLOAT
import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import bullet.wo
import bullet.collision.collisionDispatch.CollisionObject.AnisotropicFrictionFlags as AFF
import bullet.collision.collisionDispatch.CollisionObject.CollisionFlags as CF
import bullet.collision.collisionDispatch.CollisionObject.CollisionObjectTypes as Cot


//island management, m_activationState1
val ACTIVE_TAG = 1
val ISLAND_SLEEPING = 2
val WANTS_DEACTIVATION = 3
val DISABLE_DEACTIVATION = 4
val DISABLE_SIMULATION = 5


/** CollisionObject can be used to manage collision detection objects.
 *  CollisionObject maintains all information that is needed for a collision detection: Shape, Transform and AABB proxy.
 *  They can be added to the CollisionWorld.  */
open class CollisionObject {

    protected val worldTransform_ = Transform().apply { setIdentity() }
    fun setWorldTransform(worldTrans: Transform) {
        updateRevision++
        worldTransform_ put worldTrans
    }

    fun getWorldTransform() = worldTransform_


    /** interpolationWorldTransform is used for CCD and interpolation
     *  it can be either previous or future (predicted) transform   */
    protected val interpolationWorldTransform_ = Transform().apply { setIdentity() }

    fun setInterpolationWorldTransform(trans: Transform) {
        updateRevision++
        interpolationWorldTransform_ put trans
    }

    fun getInterpolationWorldTransform() = interpolationWorldTransform_


    /** those two are experimental: just added for bullet time effect, so you can still apply impulses (directly modifying
     *  velocities) without destroying the continuous interpolated motion (which uses this interpolation velocities)  */
    protected val interpolationLinearVelocity_ = Vec3()

    fun setInterpolationLinearVelocity(linVel: Vec3) {
        updateRevision++
        interpolationLinearVelocity_ put linVel
    }

    fun getInterpolationLinearVelocity() = interpolationLinearVelocity_


    protected val interpolationAngularVelocity_ = Vec3()
    fun setInterpolationAngularVelocity(angVel: Vec3) {
        updateRevision++
        interpolationAngularVelocity_ put angVel
    }

    fun getInterpolationAngularVelocity() = interpolationAngularVelocity_


    val anisotropicFriction = Vec3(1f)
    protected var hasAnisotropicFriction = AFF.DISABLED
    /** the constraint solver can discard solving contacts, if the distance is above this threshold. 0 by default.
     *  Note that using contacts with positive distance can improve stability. It increases, however, the chance of
     *  colliding with degerate contacts, such as 'interior' triangle edges */
    var contactProcessingThreshold = LARGE_FLOAT

    var broadphaseHandle: BroadphaseProxy? = null
    var collisionShape: CollisionShape? = null
        set(value) {
            updateRevision++
            field = value
            rootCollisionShape = field
        }

    /** rootCollisionShape is temporarily used to store the original collision shape
     *  The collisionShape might be temporarily replaced by a child collision shape during collision detection purposes
     *  If it is NULL, the m_collisionShape is not temporarily replaced.    */
    protected var rootCollisionShape: CollisionShape? = null

    var collisionFlags = CF.STATIC_OBJECT.i

    var islandTag = -1
    var companionId = -1
    /** index of object in world's collisionObjects array. It should be called only by CollisionWorld   */
    var worldArrayIndex = -1

    var activationState = 1
        set(value) {
            if (activationState != DISABLE_DEACTIVATION && activationState != DISABLE_SIMULATION)
                field = value
        }
    var deactivationTime = 0f

    var _friction = 0.5f
    fun setFriction(value: Float) {
        updateRevision++
        _friction = value
    }

    var _restitution = 0f
    fun setRestituition(value: Float) {
        updateRevision++
        _restitution = value
    }

    /** torsional friction orthogonal to contact normal (useful to stop spheres rolling forever)    */
    var _rollingFriction = 0f

    fun setRollingFriction(value: Float) {
        updateRevision++
        _rollingFriction = value
    }

    /** torsional friction around the contact normal (useful for grasping)  */
    var _spinningFriction = 0f

    fun setSpinningFriction(value: Float) {
        updateRevision++
        _spinningFriction = value
    }

    var contactDamping = .1f
        protected set
    var contactStiffness = 1e4f
        protected set

    /** internalType is reserved to distinguish Bullet's CollisionObject, RigidBody, SoftBody, GhostObject etc.
     *  do not assign your own internalType unless you write a new dynamics object class.   */
    var internalType = Cot.COLLISION_OBJECT.i
        protected set

    /** users can point to their objects, m_userPointer is not used by Bullet   */
    var userObjectPointer: Any? = null
    var userIndex = -1
    /** users can point to their objects, userPointer is not used by Bullet */
    var userIndex2 = -1

    /** time of impact calculation  */
    var hitFraction = 1f
    /** Swept sphere radius (0.0 by default), see ConvexConvexAlgorithm */
    var ccdSweptSphereRadius = 0f
    /** Don't do continuous collision detection if the motion (in one step) is less then ccdMotionThreshold   */
    var ccdMotionThreshold = 0f
    /** If some object should have elaborate collision filtering by sub-classes */
    protected var checkCollideWith = false

    protected var objectsWithoutCollisionCheck = ArrayList<CollisionObject>()
    /** internal update revision number. It will be increased when the object changes. This allows some subsystems
     *  to perform lazy evaluation. */
    var updateRevision = 0
        protected set

    var customDebugColorRGB: Vec3? = Vec3()
        get() = customDebugColorRGB.takeIf { collisionFlags has CF.HAS_CUSTOM_DEBUG_RENDERING_COLOR.i }
        set(value) {
            if (value == null)
                collisionFlags = collisionFlags wo CF.HAS_CUSTOM_DEBUG_RENDERING_COLOR.i
            else {
                collisionFlags = collisionFlags or CF.HAS_CUSTOM_DEBUG_RENDERING_COLOR.i
                field = value
            }
        }


    // ---------------------------------------------------- public ----------------------------------------------------
    enum class CollisionFlags {
        STATIC_OBJECT,
        KINEMATIC_OBJECT,
        NO_CONTACT_RESPONSE,
        /** this allows per-triangle material (friction/restitution)    */
        CUSTOM_MATERIAL_CALLBACK,
        CHARACTER_OBJECT,
        /** disable debug drawing   */
        DISABLE_VISUALIZE_OBJECT,
        /** disable parallel/SPU processing */
        DISABLE_SPU_COLLISION_PROCESSING,
        HAS_CONTACT_STIFFNESS_DAMPING,
        HAS_CUSTOM_DEBUG_RENDERING_COLOR,
        HAS_FRICTION_ANCHOR,
        HAS_COLLISION_SOUND_TRIGGER;

        val i = 1 shl ordinal
    }

    infix fun Int.or(b: CollisionFlags) = or(b.i)
    infix fun Int.wo(b: CollisionFlags) = and(b.i.inv())

    enum class CollisionObjectTypes {
        COLLISION_OBJECT,
        RIGID_BODY,
        /** GHOST_OBJECT keeps track of all objects overlapping its AABB and that pass its collision filter
         *  It is useful for collision sensors, explosion objects, character controller etc.    */
        GHOST_OBJECT,
        SOFT_BODY,
        HF_FLUID,
        USER_TYPE,
        FEATHERSTONE_LINK;

        val i = 1 shl ordinal
    }

    companion object {
        infix fun Int.or(b: CollisionObjectTypes) = or(b.i)
        infix fun Int.has(b: CollisionObjectTypes) = (this and b.i) != 0
        infix fun Int.hasnt(b: CollisionObjectTypes) = (this and b.i) == 0
    }

    enum class AnisotropicFrictionFlags { DISABLED, NORMAL, ROLLING;

        val i = ordinal
    }

    /** static objects, kinematic and object without contact response don't merge islands   */
    fun mergesSimulationIslands() = collisionFlags hasnt (CF.STATIC_OBJECT.i or CF.KINEMATIC_OBJECT.i or CF.NO_CONTACT_RESPONSE.i)

    fun setAnisotropicFriction(anisotropicFriction: Vec3, frictionMode: AFF = AFF.NORMAL) {
        anisotropicFriction put anisotropicFriction
        val isUnity = anisotropicFriction[0] != 1f || anisotropicFriction[1] != 1f || anisotropicFriction[2] != 1f
        hasAnisotropicFriction = if (isUnity) frictionMode else AFF.DISABLED
    }

    fun hasAnisotropicFriction(frictionMode: AFF = AFF.NORMAL) = hasAnisotropicFriction(frictionMode.i)
    fun hasAnisotropicFriction(frictionMode: Int = AFF.NORMAL.i) = hasAnisotropicFriction.i has frictionMode
    val isStaticObject get() = collisionFlags has CF.STATIC_OBJECT.i
    val isKinematicObject get() = collisionFlags has CF.KINEMATIC_OBJECT.i
    val isStaticOrKinematicObject get() = collisionFlags has (CF.KINEMATIC_OBJECT.i or CF.STATIC_OBJECT.i)
    val hasContactResponse get() = collisionFlags has CF.NO_CONTACT_RESPONSE.i

    fun setIgnoreCollisionCheck(co: CollisionObject, ignoreCollisionCheck: Boolean) {
        if (ignoreCollisionCheck)
        /*  We don't check for duplicates. Is it ok to leave that up to the user of this API?
            int index = m_objectsWithoutCollisionCheck.findLinearSearch(co);    */
            objectsWithoutCollisionCheck.add(co)
        else
            objectsWithoutCollisionCheck.remove(co)
        checkCollideWith = objectsWithoutCollisionCheck.isNotEmpty()
    }

    fun checkCollideWithOverride(co: CollisionObject) = !objectsWithoutCollisionCheck.contains(co)

    fun forceActiavationState(newState: Int) {
        activationState = newState
    }

    fun activate(forceActivation: Boolean = false) {
        if (forceActivation || collisionFlags hasnt (CF.STATIC_OBJECT.i or CF.KINEMATIC_OBJECT.i)) {
            activationState = ACTIVE_TAG
            deactivationTime = 0f
        }
    }

    val isActive get() = activationState != ISLAND_SLEEPING && activationState != DISABLE_SIMULATION

    fun setContactStiffnessAndDamping(stiffness: Float, damping: Float) {
        updateRevision++
        contactStiffness = stiffness
        contactDamping = damping

        collisionFlags = collisionFlags or CF.HAS_CONTACT_STIFFNESS_DAMPING.i

        //avoid divisions by zero...
        if (contactStiffness < Float.EPSILON) contactStiffness = Float.EPSILON
    }

    val ccdSquareMotionThreshold get() = ccdMotionThreshold * ccdMotionThreshold

    fun checkCollideWith(co: CollisionObject) = if (checkCollideWith) checkCollideWithOverride(co) else true
}