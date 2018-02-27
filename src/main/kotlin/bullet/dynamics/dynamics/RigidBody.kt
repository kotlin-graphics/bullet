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

package bullet.dynamics.dynamics

import bullet.collision.collisionDispatch.CollisionObject
import bullet.collision.collisionShapes.CollisionShape
import bullet.dynamics.constraintSolver.TypedConstraint
import bullet.linearMath.*
import bullet.push
import kotlin.math.pow
import kotlin.math.sqrt
import bullet.collision.collisionDispatch.CollisionObject.CollisionFlags as Cf
import bullet.collision.collisionDispatch.CollisionObject.CollisionObjectTypes as Cot

// 'temporarily' global variables
val gDeactivationTime = 2f
var gDisableDeactivation = false
var uniqueId = 0

//island management, m_activationState1
val ACTIVE_TAG = 1
val ISLAND_SLEEPING = 2
val WANTS_DEACTIVATION = 3
val DISABLE_DEACTIVATION = 4
val DISABLE_SIMULATION = 5

enum class RigidBodyFlags(val i: Int) {
    DISABLE_WORLD_GRAVITY(1),
    /*  ENABLE_GYROPSCOPIC_FORCE flags is enabled by default in Bullet 2.83 and onwards.
     *  And it ENABLE_GYROPSCOPIC_FORCE becomes equivalent to ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY
     *  See Demos/GyroscopicDemo and computeGyroscopicImpulseImplicit   */
    ENABLE_GYROSCOPIC_FORCE_EXPLICIT(2),
    ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD(4),
    ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY(8),
    ENABLE_GYROPSCOPIC_FORCE(ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY.i)
}

infix fun Int.or(b: RigidBodyFlags) = or(b.i)
infix fun Int.has(b: RigidBodyFlags) = (this and b.i) != 0
infix fun Int.hasnt(b: RigidBodyFlags) = (this and b.i) == 0

/** The RigidBody is the main class for rigid body objects. It is derived from CollisionObject, so it keeps a pointer
 *  to a CollisionShape.
 *  It is recommended for performance and memory use to share CollisionShape objects whenever possible.
 *  There are 3 types of rigid bodies:
 *  - A) Dynamic rigid bodies, with positive mass. Motion is controlled by rigid body dynamics.
 *  - B) Fixed objects with zero mass. They are not moving (basically collision objects)
 *  - C) Kinematic objects, which are objects without mass, but the user can move them. There is on-way interaction,
 *  and Bullet calculates a velocity based on the timestep and previous and current world transform.
 *  Bullet automatically deactivates dynamic rigid bodies, when the velocity is below a threshold for a given time.
 *  Deactivated (sleeping) rigid bodies don't take any processing time, except a minor broadphase collision detection
 *  impact (to allow active objects to activate/wake up sleeping objects)   */
class RigidBody(constructionInfo: RigidBodyConstructionInfo) : CollisionObject() {

    constructor(mass: Float, motionState: MotionState?, collisionShape: CollisionShape?, localInertia: Vec3 = Vec3()) :
            this(RigidBodyConstructionInfo(mass, motionState, null, localInertia))

    init {
        internalType = Cot.RIGID_BODY.i
    }

    val invInertiaTensorWorld = Mat3()

    val _linearVelocity = Vec3()
    fun setLinearVelocity(linVel: Vec3) {
        updateRevision++
        _linearVelocity put linVel
    }

    val _angularVelocity = Vec3()
    fun setAngularVelocity(angVel: Vec3) {
        updateRevision++
        _angularVelocity put angVel
    }

    var inverseMass = 0f
    var linearFactor = Vec3(1f)
        set(value) {
            field put value
            invMass put field * inverseMass
        }

    val _gravity = Vec3()
    val gravityAcceleration = Vec3()
    val invInertiaLocal = Vec3()
    val totalForce = Vec3()
    val totalTorque = Vec3()

    var linearDamping = clamped(constructionInfo.linearDamping, 0f, 1f)
    var angularDamping = clamped(constructionInfo.angularDamping, 0f, 1f)

    val additionalDamping = constructionInfo.additionalDamping
    val additionalDampingFactor = constructionInfo.additionalDampingFactor
    val additionalLinearDampingThresholdSqr = constructionInfo.additionalLinearDampingThresholdSqr
    val additionalAngularDampingThresholdSqr = constructionInfo.additionalAngularDampingThresholdSqr
    val additionalAngularDampingFactor = constructionInfo.additionalAngularDampingFactor

    var linearSleepingThreshold = constructionInfo.linearSleepingThreshold
    var angularSleepingThreshold = constructionInfo.angularSleepingThreshold

    /** optionalMotionState allows to automatic synchronize the world transform for active objects  */
    private var optionalMotionState = constructionInfo.motionState

    init {
        if (optionalMotionState != null) optionalMotionState!!.getWorldTransform(worldTransform_)
        else worldTransform_ put constructionInfo.startWorldTransform

        interpolationWorldTransform_ put worldTransform_
        interpolationLinearVelocity_ put 0f
        interpolationAngularVelocity_ put 0f

        // moved to CollisionObject
        _friction = constructionInfo.friction
        _rollingFriction = constructionInfo.rollingFriction
        _spinningFriction = constructionInfo.spinningFriction

        _restitution = constructionInfo.restitution

        collisionShape = constructionInfo.collisionShape
    }

    /** keep track of typed constraints referencing this rigid body, to disable collision between linked bodies */
    val constraintRefs = ArrayList<TypedConstraint>()

    var rigidbodyFlags = RigidBodyFlags.ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY.i

    var debugBodyId = uniqueId++

    val deltaLinearVelocity = Vec3()
    val deltaAngularVelocity = Vec3()
    val _angularFactor = Vec3(1f)
    val invMass = inverseMass * linearFactor
    init {
        setMassProps(constructionInfo.mass, constructionInfo.localInertia)
        updateInertiaTensor()
    }
    val pushVelocity = Vec3()
    val turnVelocity = Vec3()

    /** The RigidBodyConstructionInfo structure provides information to create a rigid body. Setting mass to zero
     *  creates a fixed (non-dynamic) rigid body.
     *  For dynamic objects, you can use the collision shape to approximate the local inertia tensor, otherwise use
     *  the zero vector (default argument)
     *  You can use the motion state to synchronize the world transform between physics and graphics objects.
     *  And if the motion state is provided, the rigid body will initialize its initial world transform from
     *  the motion state, startWorldTransform is only used when you don't provide a motion state.   */
    class RigidBodyConstructionInfo(
            val mass: Float,
            /** When a motionState is provided, the rigid body will initialize its world transform from the motion state
             *  In this case, startWorldTransform is ignored. */
            val motionState: MotionState?,
            val collisionShape: CollisionShape?,
            val localInertia: Vec3
    ) {
        var startWorldTransform = Transform().apply { setIdentity() }

        var linearDamping = 0f
        var angularDamping = 0f

        /** best simulation results when friction is non-zero   */
        var friction = 0.5f
        /** The rollingFriction prevents rounded shapes, such as spheres, cylinders and capsules from rolling forever.
         *  See Bullet/Demos/RollingFrictionDemo for usage  */
        var rollingFriction = 0f
        /** torsional friction around contact normal    */
        var spinningFriction = 0f

        /** best simulation results using zero restitution. */
        var restitution = 0f

        var linearSleepingThreshold = 0.8f
        var angularSleepingThreshold = 1f

        /** Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
         *  Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics system
         *  has improved, this should become obsolete   */
        var additionalDamping = false
        var additionalDampingFactor = 0.005f
        var additionalLinearDampingThresholdSqr = 0.01f
        var additionalAngularDampingThresholdSqr = 0.01f
        var additionalAngularDampingFactor = 0.01f
    }

    fun proceedToTransform(newTrans: Transform) = setCenterOfMassTransform(newTrans)

    companion object {
        /** to keep collision detection and dynamics separate we don't store a rigidbody pointer but a rigidbody is derived
         *  from CollisionObject, so we can safely perform an upcast    */
        fun upcast(colObj: CollisionObject): RigidBody? = if (colObj.internalType has Cot.RIGID_BODY) colObj as RigidBody else null
    }

    /** continuous collision detection needs prediction */
    fun predictIntegratedTransform(timeStep: Float, predictedTransform: Transform) =
            TransformUtil.integrateTransform(worldTransform_, _linearVelocity, _angularVelocity, timeStep, predictedTransform)

    fun saveKinematicState(timeStep: Float) {
        //todo: clamp to some (user definable) safe minimum timestep, to limit maximum angular/linear velocities
        if (timeStep != 0f) {
            //if we use motionstate to synchronize world transforms, get the new kinematic/animated world transform
            optionalMotionState?.getWorldTransform(worldTransform_)
            val linVel = Vec3()
            val angVel = Vec3()

            TransformUtil.calculateVelocity(interpolationWorldTransform_, worldTransform_, timeStep, _linearVelocity, _angularVelocity)
            interpolationLinearVelocity_ put _linearVelocity
            interpolationAngularVelocity_ put _angularVelocity
            interpolationWorldTransform_ put worldTransform_
            //printf("angular = %f %f %f\n",m_angularVelocity.getX(),m_angularVelocity.getY(),m_angularVelocity.getZ());
        }
    }

    fun applyGravity() {
        if (isStaticOrKinematicObject) return
        applyCentralForce(_gravity)
    }

    var gravity
        get() = gravityAcceleration
        set(value) {
            if (inverseMass != 0f)
                _gravity put value * (1f / inverseMass)
            gravityAcceleration put value
        }

    fun setDamping(linDamping: Float, angDamping: Float) {
        linearDamping = clamped(linDamping, 0f, 1f)
        angularDamping = clamped(angDamping, 0f, 1f)
    }

    /** applyDamping damps the velocity, using the given m_linearDamping and m_angularDamping   */
    fun applyDamping(timeStep: Float) {
        /*  On new damping: see discussion/issue report here: http://code.google.com/p/bullet/issues/detail?id=74
            todo: do some performance comparisons (but other parts of the engine are probably bottleneck anyway         */

        _linearVelocity *= (1f - linearDamping).pow(timeStep)
        _angularVelocity *= (1 - angularDamping).pow(timeStep)

        if (additionalDamping) {
            /*  Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
                Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics system
                has improved, this should become obsolete             */
            if (_angularVelocity.length2() < additionalAngularDampingThresholdSqr &&
                    _linearVelocity.length2() < additionalLinearDampingThresholdSqr) {
                _angularVelocity *= additionalDampingFactor
                _linearVelocity *= additionalDampingFactor
            }

            val speed = _linearVelocity.length()
            if (speed < linearDamping) {
                val dampVel = 0.005f
                if (speed > dampVel) {
                    val dir = _linearVelocity.normalized()
                    _linearVelocity -= dir * dampVel
                } else
                    _linearVelocity put 0f
            }

            val angSpeed = _angularVelocity.length()
            if (angSpeed < angularDamping) {
                val angDampVel = 0.005f
                if (angSpeed > angDampVel) {
                    val dir = _angularVelocity.normalized()
                    _angularVelocity -= dir * angDampVel
                } else
                    _angularVelocity put 0f
            }
        }
    }

    fun setMassProps(mass: Float, inertia: Vec3) {
        if (mass == 0f) {
            collisionFlags = collisionFlags or Cf.STATIC_OBJECT
            inverseMass = 0f
        } else {
            collisionFlags = collisionFlags wo Cf.STATIC_OBJECT
            inverseMass = 1f / mass
        }

        //Fg = m * a
        _gravity put mass * gravityAcceleration

        invInertiaLocal.put(
                if (inertia.x != 0f) 1f / inertia.x else 0f,
                if (inertia.y != 0f) 1f / inertia.y else 0f,
                if (inertia.z != 0f) 1f / inertia.z else 0f)

        invMass put linearFactor * inverseMass
    }

    fun integrateVelocities(step: Float) {
        if (isStaticOrKinematicObject) return

        _linearVelocity += totalForce * (inverseMass * step)
        _angularVelocity += invInertiaTensorWorld * totalTorque * step

        // clamp angular velocity. collision calculations will fail on higher angular velocities
        val angVel = _angularVelocity.length()
        if (angVel * step > HALF_PI)
            _angularVelocity *= (HALF_PI / step) / angVel
    }

    fun setCenterOfMassTransform(xform: Transform) {
        interpolationWorldTransform_ put if (isKinematicObject) worldTransform_ else xform
        interpolationLinearVelocity_ put _linearVelocity
        interpolationAngularVelocity_ put _angularVelocity
        worldTransform_ put xform
        updateInertiaTensor()
    }

    fun applyCentralForce(force: Vec3) = totalForce plusAssign (force * linearFactor)

    fun setSleepingThresholds(linear: Float, angular: Float) {
        linearSleepingThreshold = linear
        angularSleepingThreshold = angular
    }

    fun applyTorque(torque: Vec3) = totalTorque plusAssign (torque * _angularFactor)

    fun applyForce(force: Vec3, relPos: Vec3) {
        applyCentralForce(force)
        applyTorque(relPos cross (force * linearFactor))
    }

    fun applyCentralImpulse(impulse: Vec3) = _linearVelocity plusAssign (impulse * linearFactor * inverseMass)

    fun applyTorqueImpulse(torque: Vec3) = _angularVelocity plusAssign (invInertiaTensorWorld * torque * _angularFactor)

    fun applyImpulse(impulse: Vec3, relPos: Vec3) {
        if (inverseMass != 0f) {
            applyCentralImpulse(impulse)
            // TODO if (angularFactor)
            applyTorqueImpulse(relPos cross (impulse * linearFactor))
        }
    }

    fun clearForces() {
        totalForce put 0f
        totalTorque put 0f
    }

    fun updateInertiaTensor() = invInertiaTensorWorld put worldTransform_.basis.scaled(invInertiaLocal) * worldTransform_.basis.transpose()

    val centerOfMassPosition get() = worldTransform_.origin
    val orientation
        get():Quat {
            val orn = Quat()
            worldTransform_.basis.getRotation(orn)
            return orn
        }

    val centerOfMassTransform get() = worldTransform_


    fun getVelocityInLocalPoint(relPos: Vec3) =
            // we also calculate lin/ang velocity for kinematic objects
            _linearVelocity + (_angularVelocity cross relPos)
    // for kinematic objects, we could also use use:
    //		return 	(m_worldTransform(rel_pos) - m_interpolationWorldTransform(rel_pos)) / m_kinematicTimeStep;

    fun translate(v: Vec3) = worldTransform_.origin plusAssign v

    fun getAabb(aabbMin: Vec3, aabbMax: Vec3) = collisionShape!!.getAabb(worldTransform_, aabbMin, aabbMax)

    fun computeImpulseDenominator(pos: Vec3, normal: Vec3): Float {
        val r0 = pos - centerOfMassPosition
        val c0 = r0 cross normal
        val vec = (c0 * invInertiaTensorWorld) cross r0
        return inverseMass + (normal dot vec)
    }

    fun computeAngularImpulseDenominator(axis: Vec3): Float {
        val vec = axis * invInertiaTensorWorld
        return axis dot vec
    }

    fun updateDeactivation(timeStep: Float) {
        if (activationState == ISLAND_SLEEPING || activationState == DISABLE_DEACTIVATION) return
        if (_linearVelocity.length2() < linearSleepingThreshold * linearSleepingThreshold &&
                _angularVelocity.length2() < angularSleepingThreshold * angularSleepingThreshold)
            deactivationTime += timeStep
        else {
            deactivationTime = 0f
            activationState = 0
        }
    }

    val wantsSleeping
        get() = when {
            activationState == DISABLE_DEACTIVATION -> false
        //disable deactivation
            gDisableDeactivation || deactivationTime == 0f -> false
            activationState == ISLAND_SLEEPING || activationState == WANTS_DEACTIVATION -> true
            deactivationTime > deactivationTime -> true
            else -> false
        }

    /** MotionState allows to automatic synchronize the world transform for active objects  */
    val motionState get() = optionalMotionState

    fun setMotionState(motionState: MotionState?) {
        optionalMotionState = motionState
        optionalMotionState?.getWorldTransform(worldTransform_)
    }

    //for experimental overriding of friction/contact solver func
    var contactSolverType = 0
    var frictionSolverType = 0

    fun setAngularFactor(angFac: Vec3) {
        updateRevision++
        _angularFactor put angFac
    }

    fun setAngularFactor(angFac: Float) {
        updateRevision++
        _angularFactor put angFac
    }

    fun getAngularFactor() = _angularFactor

    /** is this rigidbody added to a CollisionWorld/DynamicsWorld/Broadphase?   */
    val isInWorld get() = broadphaseHandle != null

    fun addConstraintRef(c: TypedConstraint) {
        // disable collision with the 'other' body
        val index = constraintRefs.indexOf(c)
        //  don't add constraints that are already referenced
        if (index == constraintRefs.size) {
            constraintRefs push c
            val colObjA = c.rbA!!
            val colObjB = c.rbB!!
            if (colObjA === this)
                colObjA.setIgnoreCollisionCheck(colObjB, true)
            else
                colObjB.setIgnoreCollisionCheck(colObjA, true)
        }
    }

    fun removeConstraintRef(c: TypedConstraint) {
        val index = constraintRefs.indexOf(c)
        //don't remove constraints that are not referenced
        if (index < constraintRefs.size) {
            constraintRefs.remove(c)
            val colObjA = c.rbA!!
            val colObjB = c.rbB!!
            if (colObjA === this)
                colObjA.setIgnoreCollisionCheck(colObjB, false)
            else
                colObjB.setIgnoreCollisionCheck(colObjA, false)
        }
    }

    /** perform implicit force computation in world space   */
    fun computeGyroscopicImpulseImplicit_World(step: Float): Vec3 {
        /*  Use full newton-euler equations.  common practice to drop the wxIw term. want it for better tumbling behavior.
            Calculate using implicit euler step so it's stable. */
        val inertiaLocal = localInertia
        val w0 = _angularVelocity

        val i = worldTransform_.basis.scaled(inertiaLocal) * worldTransform_.basis.transpose()

        /*  use newtons method to find implicit solution for new angular velocity (w')
            f(w') = -(T*step + Iw) + Iw' + w' + w'xIw'*step = 0
            df/dw' = I + 1xIw'*step + w'xI*step     */

        val w1 = Vec3(w0)
        // one step of newton's method
        run {
            val fw = evalEulerEqn(w1, w0, Vec3(), step, i)
            val dfw = evalEulerEqnDeriv(w1, w0, step, i)

            val dw = dfw solve33 fw
            //const btMatrix3x3 dfw_inv = dfw.inverse();
            //dw = dfw_inv*fw;
            w1 -= dw
        }
        return w1 - w0 // gf
    }

    fun evalEulerEqn(w1: Vec3, w0: Vec3, t: Vec3, dt: Float, i: Mat3) = i * w1 + w1.cross(i * w1) * dt - (t * dt + i * w0)

    fun evalEulerEqnDeriv(w1: Vec3, w0: Vec3, dt: Float, i: Mat3): Mat3 {

        val w1x = Mat3()
        val iw1x = Mat3()
        val iwi = i * w1
        w1.getSkewSymmetricMatrix(w1x[0], w1x[1], w1x[2])
        iwi.getSkewSymmetricMatrix(iw1x[0], iw1x[1], iw1x[2])

        return i + (w1x * i - iw1x) * dt    // dfw1
    }

    /** perform implicit force computation in body space (inertial frame)   */
    fun computeGyroscopicImpulseImplicit_Body(step: Float): Vec3 {

        val idl = localInertia
        val omega1 = _angularVelocity
        val q = getWorldTransform().getRotation()

        // Convert to body coordinates
        val omegab = q.inverse() rotate omega1
        val ib = Mat3()
        ib.put(idl.x, 0f, 0f,
                0f, idl.y, 0f,
                0f, 0f, idl.z)

        val ibo = ib * omegab

        // Residual vector
        val f = step * omegab.cross(ibo)

        val skew0 = Mat3()
        omegab.getSkewSymmetricMatrix(skew0[0], skew0[1], skew0[2])
        val om = ib * omegab
        val skew1 = Mat3()
        om.getSkewSymmetricMatrix(skew1[0], skew1[1], skew1[2])

        // Jacobian
        val j = ib + (skew0 * ib - skew1) * step

//	btMatrix3x3 Jinv = j.inverse();
//	btVector3 omegaDiv = Jinv*f;
        val omegaDiv = j solve33 f

        // Single Newton-Raphson update
        omegab -= omegaDiv  // solve33(j, f)
        // Back to world coordinates
        val omega2 = q rotate omegab
        return omega2 - omega1 // gf
    }

    ///explicit version is best avoided, it gains energy
    fun computeGyroscopicForceExplicit(maxGyroscopicForce: Float): Vec3 {

        val inertiaTensorWorld = getWorldTransform().basis.scaled(localInertia) * getWorldTransform().basis.transpose()
        val tmp = inertiaTensorWorld * _angularVelocity
        val gf = _angularVelocity cross tmp
        val l2 = gf.length2()
        if (l2 > maxGyroscopicForce * maxGyroscopicForce)
            gf *= 1f / sqrt(l2) * maxGyroscopicForce
        return gf
    }

    val localInertia
        get():Vec3 {
            val inertiaLocal = Vec3()
            val inertia = invInertiaLocal
            inertiaLocal.put(
                    if (inertia.x != 0f) 1f / inertia.x else 0f,
                    if (inertia.y != 0f) 1f / inertia.y else 0f,
                    if (inertia.z != 0f) 1f / inertia.z else 0f)
            return inertiaLocal
        }
}