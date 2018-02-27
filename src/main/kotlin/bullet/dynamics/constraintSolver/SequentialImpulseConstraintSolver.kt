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

package bullet.dynamics.constraintSolver

import bullet.*
import bullet.collision.broadphaseCollision.Dispatcher
import bullet.collision.collisionDispatch.CollisionObject
import bullet.collision.narrowPhaseCollision.*
import bullet.dynamics.dynamics.RigidBody
import bullet.dynamics.dynamics.has
import bullet.linearMath.DebugDraw
import bullet.linearMath.Vec3
import bullet.linearMath.planeSpace1
import bullet.linearMath.times
import kotlin.math.abs
import kotlin.math.sqrt
import kotlin.reflect.KMutableProperty0
import bullet.collision.collisionDispatch.CollisionObject.AnisotropicFrictionFlags as Aff
import bullet.collision.narrowPhaseCollision.ContactPointFlags as Cpf
import bullet.dynamics.constraintSolver.SolverMode as Sm
import bullet.dynamics.dynamics.RigidBodyFlags as Rbf

typealias SingleConstraintRowSolver = (SolverBody, SolverBody, SolverConstraint) -> Float

var numSplitImpulseRecoveries = 0

/** The SequentialImpulseConstraintSolver is a fast SIMD implementation of the Projected Gauss Seidel (iterative LCP) method.   */
class SequentialImpulseConstraintSolver : ConstraintSolver() {

    val tmpSolverBodyPool = ArrayList<SolverBody>()
    val tmpSolverContactConstraintPool = ArrayList<SolverConstraint>()
    val tmpSolverNonContactConstraintPool = ArrayList<SolverConstraint>()
    val tmpSolverContactFrictionConstraintPool = ArrayList<SolverConstraint>()
    val tmpSolverContactRollingFrictionConstraintPool = ArrayList<SolverConstraint>()

    var orderTmpConstraintPool = intArrayOf()
    var orderNonContactConstraintPool = intArrayOf()
    var orderFrictionConstraintPool = intArrayOf()
    val tmpConstraintSizesPool = ArrayList<TypedConstraint.ConstraintInfo1>()
    var maxOverrideNumSolverIterations = 0
    var fixedBodyId = 0
    /** Only used for multithreading.
     *  When running solvers on multiple threads, a race condition exists for Kinematic objects that participate in
     *  more than one solver.
     *  The getOrInitSolverBody() function writes the companionId of each body (storing the index of the solver body
     *  for the current solver). For normal dynamic bodies it isn't an issue because they can only be in one island
     *  (and therefore one thread) at a time. But kinematic bodies can be in multiple islands at once.
     *  To avoid this race condition, this solver does not write the companionId, instead it stores the solver body
     *  index in this solver-local table, indexed by the uniqueId of the body.  */
    val kinematicBodyUniqueIdToSolverBodyTable = ArrayList<Int>()

    var resolveSingleConstraintRowGeneric: SingleConstraintRowSolver? = null
    var resolveSingleConstraintRowLowerLimit: SingleConstraintRowSolver? = null
    var resolveSplitPenetrationImpulse: SingleConstraintRowSolver? = null
    var cachedSolverMode = 0   // used to check if SOLVER_SIMD flag has been changed

    fun setupSolverFunctions(useSimd: Boolean) {
        resolveSingleConstraintRowGeneric = ::resolveSingleConstraintRowGeneric
        resolveSingleConstraintRowLowerLimit = ::resolveSingleConstraintRowLowerLimit
        resolveSplitPenetrationImpulse = ::resolveSplitPenetrationImpulse
    }

    var leastSquaresResidual = 0f

    init {
        setupSolverFunctions(false)
    }

    fun setupFrictionConstraint(solverConstraint: SolverConstraint, normalAxis: Vec3, solverBodyIdA: Int, solverBodyIdB: Int,
                                cp: ManifoldPoint, relPos1: Vec3, relPos2: Vec3,
                                colObj0: CollisionObject, colObj1: CollisionObject, relaxation: Float,
                                infoGlobal: ContactSolverInfo,
                                desiredVelocity: Float = 0f, cfmSlip: Float = 0f) {

        val solverBodyA = tmpSolverBodyPool[solverBodyIdA]
        val solverBodyB = tmpSolverBodyPool[solverBodyIdB]

        val body0 = tmpSolverBodyPool[solverBodyIdA].originalBody
        val body1 = tmpSolverBodyPool[solverBodyIdB].originalBody

        solverConstraint.solverBodyIdA = solverBodyIdA
        solverConstraint.solverBodyIdB = solverBodyIdB

        solverConstraint.friction = cp.combinedFriction
        solverConstraint.originalContactPoint = null

        solverConstraint.appliedImpulse = 0f
        solverConstraint.appliedPushImpulse = 0f

        if (body0 != null) {
            solverConstraint.contactNormal1 put normalAxis
            val ftorqueAxis1 = relPos1 cross solverConstraint.contactNormal1
            solverConstraint.relpos1CrossNormal put ftorqueAxis1
            solverConstraint.angularComponentA put body0.invInertiaTensorWorld * ftorqueAxis1 * body0.getAngularFactor() // TODO check order
        } else {
            solverConstraint.contactNormal1 put 0f
            solverConstraint.relpos1CrossNormal put 0f
            solverConstraint.angularComponentA put 0f
        }
        if (body1 != null) {
            solverConstraint.contactNormal2 put -normalAxis
            val ftorqueAxis1 = relPos2 cross solverConstraint.contactNormal2
            solverConstraint.relpos2CrossNormal put ftorqueAxis1
            solverConstraint.angularComponentB put (body1.invInertiaTensorWorld) * ftorqueAxis1 * body1.getAngularFactor()
        } else {
            solverConstraint.contactNormal2 put 0f
            solverConstraint.relpos2CrossNormal put 0f
            solverConstraint.angularComponentB put 0f
        }

        run {
            val vec = Vec3()
            var denom0 = 0f
            var denom1 = 0f
            body0?.let {
                vec put solverConstraint.angularComponentA.cross(relPos1)
                denom0 = it.inverseMass + normalAxis.dot(vec)
            }
            body1?.let {
                vec put (-solverConstraint.angularComponentB).cross(relPos2)
                denom1 = it.inverseMass + normalAxis.dot(vec)
            }
            val denom = relaxation / (denom0 + denom1)
            solverConstraint.jacDiagABInv = denom
        }

        run {
            val vel1Dotn = solverConstraint.contactNormal1.dot(if (body0 != null) solverBodyA.linearVelocity + solverBodyA.externalForceImpulse else Vec3()) +
                    solverConstraint.relpos1CrossNormal.dot(if (body0 != null) solverBodyA.angularVelocity else Vec3())
            val vel2Dotn = solverConstraint.contactNormal2.dot(if (body1 != null) solverBodyB.linearVelocity + solverBodyB.externalForceImpulse else Vec3()) +
                    solverConstraint.relpos2CrossNormal.dot(if (body1 != null) solverBodyB.angularVelocity else Vec3())

            val relVel = vel1Dotn + vel2Dotn

//		btScalar positionalError = 0.f;

            val velocityError = desiredVelocity - relVel
            val velocityImpulse = velocityError * solverConstraint.jacDiagABInv

            var penetrationImpulse = 0f

            if (cp.contactPointFlags has Cpf.FRICTION_ANCHOR.i) {
                val distance = (cp.positionWorldOnA - cp.positionWorldOnB) dot normalAxis
                val positionalError = -distance * infoGlobal.frictionERP / infoGlobal.timeStep
                penetrationImpulse = positionalError * solverConstraint.jacDiagABInv
            }

            solverConstraint.rhs = penetrationImpulse + velocityImpulse
            solverConstraint.rhsPenetration = 0f
            solverConstraint.cfm = cfmSlip
            solverConstraint.lowerLimit = -solverConstraint.friction
            solverConstraint.upperLimit = solverConstraint.friction
        }
    }

    fun setupTorsionalFrictionConstraint(solverConstraint: SolverConstraint, normalAxis1: Vec3, solverBodyIdA: Int, solverBodyIdB: Int,
                                         cp: ManifoldPoint, combinedTorsionalFriction: Float, relPos1: Vec3, relPos2: Vec3,
                                         colObj0: CollisionObject, colObj1: CollisionObject, relaxation: Float,
                                         desiredVelocity: Float = 0f, cfmSlip: Float = 0f) {

        val normalAxis = Vec3()

        solverConstraint.contactNormal1 put normalAxis
        solverConstraint.contactNormal2 put -normalAxis
        val solverBodyA = tmpSolverBodyPool[solverBodyIdA]
        val solverBodyB = tmpSolverBodyPool[solverBodyIdB]

        val body0 = tmpSolverBodyPool[solverBodyIdA].originalBody
        val body1 = tmpSolverBodyPool[solverBodyIdB].originalBody

        solverConstraint.solverBodyIdA = solverBodyIdA
        solverConstraint.solverBodyIdB = solverBodyIdB

        solverConstraint.friction = combinedTorsionalFriction
        solverConstraint.originalContactPoint = null

        solverConstraint.appliedImpulse = 0f
        solverConstraint.appliedPushImpulse = 0f

        run {
            val ftorqueAxis1 = -normalAxis1
            solverConstraint.relpos1CrossNormal put ftorqueAxis1
            solverConstraint.angularComponentA put
                    if (body0 != null) body0.invInertiaTensorWorld * ftorqueAxis1 * body0.getAngularFactor() else Vec3()
        }
        run {
            val ftorqueAxis1 = normalAxis1
            solverConstraint.relpos2CrossNormal put ftorqueAxis1
            solverConstraint.angularComponentB put
                    if (body1 != null) body1.invInertiaTensorWorld * ftorqueAxis1 * body1.getAngularFactor() else Vec3()
        }

        run {
            val iMJaA = if (body0 != null) body0.invInertiaTensorWorld * solverConstraint.relpos1CrossNormal else Vec3()
            val iMJaB = if (body1 != null) body1.invInertiaTensorWorld * solverConstraint.relpos2CrossNormal else Vec3()
            val sum = iMJaA.dot(solverConstraint.relpos1CrossNormal) + iMJaB.dot(solverConstraint.relpos2CrossNormal)
            solverConstraint.jacDiagABInv = 1f / sum
        }

        run {
            val vel1Dotn = solverConstraint.contactNormal1.dot(if (body0 != null) solverBodyA.linearVelocity + solverBodyA.externalForceImpulse else Vec3()) +
                    (solverConstraint.relpos1CrossNormal dot if (body0 != null) solverBodyA.angularVelocity else Vec3())
            val vel2Dotn = solverConstraint.contactNormal2.dot(if (body1 != null) solverBodyB.linearVelocity + solverBodyB.externalForceImpulse else Vec3()) +
                    (solverConstraint.relpos2CrossNormal dot if (body1 != null) solverBodyB.angularVelocity else Vec3())

            val relVel = vel1Dotn + vel2Dotn

//		btScalar positionalError = 0.f;

            val velocityError = desiredVelocity - relVel
            val velocityImpulse = velocityError * solverConstraint.jacDiagABInv
            solverConstraint.rhs = velocityImpulse
            solverConstraint.cfm = cfmSlip
            solverConstraint.lowerLimit = -solverConstraint.friction
            solverConstraint.upperLimit = solverConstraint.friction
        }
    }

    fun addFrictionConstraint(normalAxis: Vec3, solverBodyIdA: Int, solverBodyIdB: Int, frictionIndex: Int,
                              cp: ManifoldPoint, relPos1: Vec3, relPos2: Vec3, colObj0: CollisionObject,
                              colObj1: CollisionObject, relaxation: Float, infoGlobal: ContactSolverInfo,
                              desiredVelocity: Float = 0f, cfmSlip: Float = 0f): SolverConstraint {

        val solverConstraint = SolverConstraint()
        tmpSolverContactFrictionConstraintPool.add(solverConstraint)
        solverConstraint.frictionIndex = frictionIndex
        setupFrictionConstraint(solverConstraint, normalAxis, solverBodyIdA, solverBodyIdB, cp, relPos1, relPos2,
                colObj0, colObj1, relaxation, infoGlobal, desiredVelocity, cfmSlip)
        return solverConstraint
    }

    fun addTorsionalFrictionConstraint(normalAxis: Vec3, solverBodyIdA: Int, solverBodyIdB: Int,
                                       frictionIndex: Int, cp: ManifoldPoint, combinedTorsionalFriction: Float,
                                       relPos1: Vec3, relPos2: Vec3, colObj0: CollisionObject,
                                       colObj1: CollisionObject, relaxation: Float, desiredVelocity: Float = 0f,
                                       cfmSlip: Float = 0f): SolverConstraint {

        val solverConstraint = SolverConstraint()
        tmpSolverContactRollingFrictionConstraintPool.add(solverConstraint)
        solverConstraint.frictionIndex = frictionIndex
        setupTorsionalFrictionConstraint(solverConstraint, normalAxis, solverBodyIdA, solverBodyIdB, cp,
                combinedTorsionalFriction, relPos1, relPos2, colObj0, colObj1, relaxation, desiredVelocity, cfmSlip)
        return solverConstraint
    }

    /** JVM specific, @return relaxation    */
    fun setupContactConstraint(solverConstraint: SolverConstraint, solverBodyIdA: Int, solverBodyIdB: Int,
                               cp: ManifoldPoint, infoGlobal: ContactSolverInfo, relPos1: Vec3, relPos2: Vec3): Float {

        //	const btVector3& pos1 = cp.getPositionWorldOnA();
        //	const btVector3& pos2 = cp.getPositionWorldOnB();

        val bodyA = tmpSolverBodyPool[solverBodyIdA]
        val bodyB = tmpSolverBodyPool[solverBodyIdB]

        val rb0 = bodyA.originalBody
        val rb1 = bodyB.originalBody

//			btVector3 rel_pos1 = pos1 - colObj0->getWorldTransform().getOrigin();
//			btVector3 rel_pos2 = pos2 - colObj1->getWorldTransform().getOrigin();
        //rel_pos1 = pos1 - bodyA->getWorldTransform().getOrigin();
        //rel_pos2 = pos2 - bodyB->getWorldTransform().getOrigin();

        val relaxation = infoGlobal.sor
        val invTimeStep = 1f / infoGlobal.timeStep

        //cfm = 1 /       ( dt * kp + kd )
        //erp = dt * kp / ( dt * kp + kd )

        var cfm = infoGlobal.globalCfm
        var erp = infoGlobal.erp2

        if (cp.contactPointFlags has Cpf.HAS_CONTACT_CFM || cp.contactPointFlags has Cpf.HAS_CONTACT_ERP) {
            if (cp.contactPointFlags has Cpf.HAS_CONTACT_CFM)
                cfm = cp.contactCFM
            if (cp.contactPointFlags has Cpf.HAS_CONTACT_ERP)
                erp = cp.contactERP
        } else if (cp.contactPointFlags has Cpf.CONTACT_STIFFNESS_DAMPING) {
            var denom = infoGlobal.timeStep * cp.combinedContactStiffness1 + cp.combinedContactDamping1
            if (denom < Float.EPSILON)
                denom = Float.EPSILON
            cfm = 1f / denom
            erp = (infoGlobal.timeStep * cp.combinedContactStiffness1) / denom
        }
        cfm *= invTimeStep

        val torqueAxis0 = relPos1 cross cp.normalWorldOnB
        solverConstraint.angularComponentA put if (rb0 != null) rb0.invInertiaTensorWorld * torqueAxis0 * rb0.getAngularFactor() else Vec3()
        val torqueAxis1 = relPos2 cross cp.normalWorldOnB
        solverConstraint.angularComponentB put if (rb1 != null) rb1.invInertiaTensorWorld * -torqueAxis1 * rb1.getAngularFactor() else Vec3()

        run {
            val vec = Vec3()
            var denom0 = 0f
            var denom1 = 0f
            rb0?.let {
                vec put solverConstraint.angularComponentA.cross(relPos1)
                denom0 = it.inverseMass + cp.normalWorldOnB.dot(vec)
            }
            rb1?.let {
                vec put (-solverConstraint.angularComponentB).cross(relPos2)
                denom1 = it.inverseMass + cp.normalWorldOnB.dot(vec)  // TODO order
            }

            val denom = relaxation / (denom0 + denom1 + cfm)
            solverConstraint.jacDiagABInv = denom
        }

        if (rb0 != null) {
            solverConstraint.contactNormal1 put cp.normalWorldOnB
            solverConstraint.relpos1CrossNormal put torqueAxis0
        } else {
            solverConstraint.contactNormal1 put 0f
            solverConstraint.relpos1CrossNormal put 0f
        }
        if (rb1 != null) {
            solverConstraint.contactNormal2 put -cp.normalWorldOnB
            solverConstraint.relpos2CrossNormal put -torqueAxis1
        } else {
            solverConstraint.contactNormal2 put 0f
            solverConstraint.relpos2CrossNormal put 0f
        }

        var restitution = 0f
        val penetration = cp.distance + infoGlobal.linearSlop

        run {
            val vel1 = rb0?.getVelocityInLocalPoint(relPos1) ?: Vec3()
            val vel2 = rb1?.getVelocityInLocalPoint(relPos2) ?: Vec3()

            //			btVector3 vel2 = rb1 ? rb1->getVelocityInLocalPoint(rel_pos2) : btVector3(0,0,0);
            val vel = vel1 - vel2
            val relVel = cp.normalWorldOnB dot vel

            solverConstraint.friction = cp.combinedFriction

            restitution = restitutionCurve(relVel, cp.combinedRestitution, infoGlobal.restitutionVelocityThreshold)
            if (restitution <= 0f) restitution = 0f
        }
        // warm starting (or zero if disabled)
        if (infoGlobal.solverMode has Sm.USE_WARMSTARTING) {
            solverConstraint.appliedImpulse = cp.appliedImpulse * infoGlobal.warmstartingFactor
            if (rb0 != null)
                bodyA.internalApplyImpulse(solverConstraint.contactNormal1 * bodyA.invMass * rb0.linearFactor,
                        solverConstraint.angularComponentA, solverConstraint.appliedImpulse)
            if (rb1 != null)
                bodyB.internalApplyImpulse(-solverConstraint.contactNormal2 * bodyB.invMass * rb1.linearFactor,
                        -solverConstraint.angularComponentB, -solverConstraint.appliedImpulse)
        } else
            solverConstraint.appliedImpulse = 0f

        solverConstraint.appliedPushImpulse = 0f

        run {
            val externalForceImpulseA = if (bodyA.originalBody != null) bodyA.externalForceImpulse else Vec3()
            val externalTorqueImpulseA = if (bodyA.originalBody != null) bodyA.externalTorqueImpulse else Vec3()
            val externalForceImpulseB = if (bodyB.originalBody != null) bodyB.externalForceImpulse else Vec3()
            val externalTorqueImpulseB = if (bodyB.originalBody != null) bodyB.externalTorqueImpulse else Vec3()

            val vel1Dotn = solverConstraint.contactNormal1.dot((bodyA.linearVelocity + externalForceImpulseA)) +
                    solverConstraint.relpos1CrossNormal.dot(bodyA.angularVelocity + externalTorqueImpulseA)
            val vel2Dotn = solverConstraint.contactNormal2.dot(bodyB.linearVelocity + externalForceImpulseB) +
                    solverConstraint.relpos2CrossNormal.dot(bodyB.angularVelocity + externalTorqueImpulseB)
            val relVel = vel1Dotn + vel2Dotn

            var positionalError = 0f
            var velocityError = restitution - relVel// * damping;

            if (penetration > 0) {
                positionalError = 0f
                velocityError -= penetration * invTimeStep
            } else
                positionalError = -penetration * erp * invTimeStep

            val penetrationImpulse = positionalError * solverConstraint.jacDiagABInv
            val velocityImpulse = velocityError * solverConstraint.jacDiagABInv

            if (!infoGlobal.splitImpulse || penetration > infoGlobal.splitImpulsePenetrationThreshold) {    //combine position and velocity into rhs
                solverConstraint.rhs = penetrationImpulse + velocityImpulse//-solverConstraint.m_contactNormal1.dot(bodyA->m_externalForce*bodyA->m_invMass-bodyB->m_externalForce/bodyB->m_invMass)*solverConstraint.m_jacDiagABInv;
                solverConstraint.rhsPenetration = 0f
            } else {    //split position and velocity into rhs and m_rhsPenetration
                solverConstraint.rhs = velocityImpulse
                solverConstraint.rhsPenetration = penetrationImpulse
            }
            solverConstraint.cfm = cfm * solverConstraint.jacDiagABInv
            solverConstraint.lowerLimit = 0f
            solverConstraint.upperLimit = 1e10f
        }
        return relaxation
    }

    fun applyAnisotropicFriction(colObj: CollisionObject?, frictionDirection: Vec3, frictionMode: CollisionObject.AnisotropicFrictionFlags) {    // TODO static?
        if (colObj != null && colObj.hasAnisotropicFriction(frictionMode)) {
            // transform to local coordinates
            val locLateral = frictionDirection * colObj.getWorldTransform().basis
            val frictionScaling = colObj.anisotropicFriction
            //apply anisotropic friction
            locLateral *= frictionScaling
            // ... and transform it back to global coordinates
            frictionDirection put colObj.getWorldTransform().basis * locLateral // TODO order
        }
    }

    fun setFrictionConstraintImpulse(solverConstraint: SolverConstraint, solverBodyIdA: Int, solverBodyIdB: Int,
                                     cp: ManifoldPoint, infoGlobal: ContactSolverInfo) {

        val bodyA = tmpSolverBodyPool[solverBodyIdA]
        val bodyB = tmpSolverBodyPool[solverBodyIdB]

        val rb0 = bodyA.originalBody
        val rb1 = bodyB.originalBody

        run {
            val frictionConstraint1 = tmpSolverContactFrictionConstraintPool[solverConstraint.frictionIndex]
            if (infoGlobal.solverMode has Sm.USE_WARMSTARTING) {
                frictionConstraint1.appliedImpulse = cp.appliedImpulseLateral1 * infoGlobal.warmstartingFactor
                if (rb0 != null)
                    bodyA.internalApplyImpulse(frictionConstraint1.contactNormal1 * rb0.inverseMass * rb0.linearFactor,
                            frictionConstraint1.angularComponentA, frictionConstraint1.appliedImpulse)
                if (rb1 != null)
                    bodyB.internalApplyImpulse(-frictionConstraint1.contactNormal2 * rb1.inverseMass * rb1.linearFactor,
                            -frictionConstraint1.angularComponentB, -frictionConstraint1.appliedImpulse)
            } else
                frictionConstraint1.appliedImpulse = 0f
        }

        if (infoGlobal.solverMode has Sm.USE_2_FRICTION_DIRECTIONS) {
            val frictionConstraint2 = tmpSolverContactFrictionConstraintPool[solverConstraint.frictionIndex + 1]
            if (infoGlobal.solverMode has Sm.USE_WARMSTARTING) {
                frictionConstraint2.appliedImpulse = cp.appliedImpulseLateral2 * infoGlobal.warmstartingFactor
                if (rb0 != null)
                    bodyA.internalApplyImpulse(frictionConstraint2.contactNormal1 * rb0.inverseMass,
                            frictionConstraint2.angularComponentA, frictionConstraint2.appliedImpulse)
                if (rb1 != null)
                    bodyB.internalApplyImpulse(-frictionConstraint2.contactNormal2 * rb1.inverseMass,
                            -frictionConstraint2.angularComponentB, -frictionConstraint2.appliedImpulse)
            } else
                frictionConstraint2.appliedImpulse = 0f
        }
    }

    /** seed2 is used for re-arranging the constraint rows. improves convergence/quality of friction  */
    var seed2 = 0L

    fun restitutionCurve(relVel: Float, restitution: Float, velocityThreshold: Float): Float {
        //printf("rel_vel =%f\n", rel_vel);
        if (abs(relVel) < velocityThreshold)
            return 0f

        return restitution * -relVel    // rest
    }

    fun convertContacts(manifoldPtr: ArrayList<PersistentManifold>, manifoldsPtr: Int, numManifolds: Int, infoGlobal: ContactSolverInfo) {
//			btCollisionObject* colObj0=0,*colObj1=0;
        for (i in 0 until numManifolds)
            convertContact(manifoldPtr[manifoldsPtr + i], infoGlobal)
    }

    fun convertContact(manifold: PersistentManifold, infoGlobal: ContactSolverInfo) {

        val colObj0 = manifold.body0!!
        val colObj1 = manifold.body1!!

        val solverBodyIdA = getOrInitSolverBody(colObj0, infoGlobal.timeStep)
        val solverBodyIdB = getOrInitSolverBody(colObj1, infoGlobal.timeStep)

//	btRigidBody* bodyA = btRigidBody::upcast(colObj0);
//	btRigidBody* bodyB = btRigidBody::upcast(colObj1);

        val solverBodyA = tmpSolverBodyPool.getOrNull(solverBodyIdA)
        val solverBodyB = tmpSolverBodyPool.getOrNull(solverBodyIdB)

        // avoid collision response between two static objects
        if (solverBodyA == null || (solverBodyA.invMass.fuzzyZero() && (solverBodyB == null || solverBodyB.invMass.fuzzyZero())))
            return

        solverBodyB!!

        val rollingFriction = 1
        for (j in 0 until manifold.numContacts) {

            val cp = manifold.getContactPoint(j)

            if (cp.distance <= manifold.contactProcessingThreshold) {

                val frictionIndex = tmpSolverContactConstraintPool.size
                val solverConstraint = SolverConstraint()
                tmpSolverContactConstraintPool += solverConstraint
                solverConstraint.solverBodyIdA = solverBodyIdA
                solverConstraint.solverBodyIdB = solverBodyIdB

                solverConstraint.originalContactPoint = cp

                val pos1 = cp.positionWorldOnA
                val pos2 = cp.positionWorldOnB

                val relPos1 = pos1 - colObj0.getWorldTransform().origin
                val relPos2 = pos2 - colObj1.getWorldTransform().origin

                val vel1 = Vec3()
                val vel2 = Vec3()

                solverBodyA.getVelocityInLocalPointNoDelta(relPos1, vel1)
                solverBodyB.getVelocityInLocalPointNoDelta(relPos2, vel2)

                val vel = vel1 - vel2
                val relVel = cp.normalWorldOnB dot vel

                val relaxation = setupContactConstraint(solverConstraint, solverBodyIdA, solverBodyIdB, cp, infoGlobal, relPos1, relPos2)

                //  setup the friction constraints

                solverConstraint.frictionIndex = tmpSolverContactFrictionConstraintPool.size

                if (cp.combinedRollingFriction > 0f && rollingFriction > 0) {

                    addTorsionalFrictionConstraint(cp.normalWorldOnB, solverBodyIdA, solverBodyIdB, frictionIndex,
                            cp, cp.combinedSpinningFriction, relPos1, relPos2, colObj0, colObj1, relaxation)
                    val axis0 = Vec3()
                    val axis1 = Vec3()
                    planeSpace1(cp.normalWorldOnB, axis0, axis1)
                    axis0.normalize()
                    axis1.normalize()

                    applyAnisotropicFriction(colObj0, axis0, Aff.ROLLING)
                    applyAnisotropicFriction(colObj1, axis0, Aff.ROLLING)
                    applyAnisotropicFriction(colObj0, axis1, Aff.ROLLING)
                    applyAnisotropicFriction(colObj1, axis1, Aff.ROLLING)
                    if (axis0.length() > 0.001)
                        addTorsionalFrictionConstraint(axis0, solverBodyIdA, solverBodyIdB, frictionIndex, cp,
                                cp.combinedRollingFriction, relPos1, relPos2, colObj0, colObj1, relaxation)
                    if (axis1.length() > 0.001)
                        addTorsionalFrictionConstraint(axis1, solverBodyIdA, solverBodyIdB, frictionIndex, cp,
                                cp.combinedRollingFriction, relPos1, relPos2, colObj0, colObj1, relaxation)
                }

                /*  Bullet has several options to set the friction directions
                    By default, each contact has only a single friction direction that is recomputed automatically very
                    frame based on the relative linear velocity.
                    If the relative velocity it zero, it will automatically compute a friction direction.

                    You can also enable two friction directions, using the SOLVER_USE_2_FRICTION_DIRECTIONS.
                    In that case, the second friction direction will be orthogonal to both contact normal and first
                    friction direction.

                    If you choose SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION, then the friction will be
                    independent from the relative projected velocity.

                    The user can manually override the friction directions for certain contacts using a contact callback,
                    and set the cp.m_lateralFrictionInitialized to true
                    In that case, you can set the target relative motion in each friction direction (cp.contactMotion1
                    and cp.contactMotion2)
                    this will give a conveyor belt effect   */
                if (infoGlobal.solverMode hasnt Sm.ENABLE_FRICTION_DIRECTION_CACHING || cp.contactPointFlags hasnt Cpf.LATERAL_FRICTION_INITIALIZED) {
                    cp.lateralFrictionDir1 put (vel - cp.normalWorldOnB * relVel) // TODO order
                    val latRelVel = cp.lateralFrictionDir1.length2()
                    if (infoGlobal.solverMode hasnt Sm.DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION && latRelVel > Float.EPSILON) {
                        cp.lateralFrictionDir1 *= 1f / sqrt(latRelVel)
                        applyAnisotropicFriction(colObj0, cp.lateralFrictionDir1, Aff.NORMAL)
                        applyAnisotropicFriction(colObj1, cp.lateralFrictionDir1, Aff.NORMAL)
                        addFrictionConstraint(cp.lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp,
                                relPos1, relPos2, colObj0, colObj1, relaxation, infoGlobal)

                        if (infoGlobal.solverMode has Sm.USE_2_FRICTION_DIRECTIONS) {
                            cp.lateralFrictionDir2 put (cp.lateralFrictionDir1 cross cp.normalWorldOnB) // TODO order
                            cp.lateralFrictionDir2.normalize()//??
                            applyAnisotropicFriction(colObj0, cp.lateralFrictionDir2, Aff.NORMAL)
                            applyAnisotropicFriction(colObj1, cp.lateralFrictionDir2, Aff.NORMAL)
                            addFrictionConstraint(cp.lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex,
                                    cp, relPos1, relPos2, colObj0, colObj1, relaxation, infoGlobal)
                        }
                    } else {
                        planeSpace1(cp.normalWorldOnB, cp.lateralFrictionDir1, cp.lateralFrictionDir2)

                        applyAnisotropicFriction(colObj0, cp.lateralFrictionDir1, Aff.NORMAL)
                        applyAnisotropicFriction(colObj1, cp.lateralFrictionDir1, Aff.NORMAL)
                        addFrictionConstraint(cp.lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp,
                                relPos1, relPos2, colObj0, colObj1, relaxation, infoGlobal)

                        if (infoGlobal.solverMode has Sm.USE_2_FRICTION_DIRECTIONS) {
                            applyAnisotropicFriction(colObj0, cp.lateralFrictionDir2, Aff.NORMAL)
                            applyAnisotropicFriction(colObj1, cp.lateralFrictionDir2, Aff.NORMAL)
                            addFrictionConstraint(cp.lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex,
                                    cp, relPos1, relPos2, colObj0, colObj1, relaxation, infoGlobal)
                        }

                        if (infoGlobal.solverMode has Sm.USE_2_FRICTION_DIRECTIONS && infoGlobal.solverMode has Sm.DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION)
                            cp.contactPointFlags = cp.contactPointFlags or Cpf.LATERAL_FRICTION_INITIALIZED
                    }
                } else {
                    addFrictionConstraint(cp.lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp,
                            relPos1, relPos2, colObj0, colObj1, relaxation, infoGlobal, cp.contactMotion1, cp.frictionCFM)

                    if (infoGlobal.solverMode has Sm.USE_2_FRICTION_DIRECTIONS)
                        addFrictionConstraint(cp.lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex, cp,
                                relPos1, relPos2, colObj0, colObj1, relaxation, infoGlobal, cp.contactMotion2, cp.frictionCFM)

                }
                setFrictionConstraintImpulse(solverConstraint, solverBodyIdA, solverBodyIdB, cp, infoGlobal)
            }
        }
    }

    fun resolveSplitPenetrationSIMD(bodyA: SolverBody, bodyB: SolverBody, contactConstraint: SolverConstraint) =
            resolveSplitPenetrationImpulse!!(bodyA, bodyB, contactConstraint)

    fun resolveSplitPenetrationImpulseCacheFriendly(bodyA: SolverBody, bodyB: SolverBody, contactConstraint: SolverConstraint) =
            resolveSplitPenetrationImpulse!!(bodyA, bodyB, contactConstraint)

    //internal method
    fun getOrInitSolverBody(body: CollisionObject, timeStep: Float): Int {
        var solverBodyIdA = -1

        if (body.companionId >= 0) {
            //body has already been converted
            solverBodyIdA = body.companionId
            assert(solverBodyIdA < tmpSolverBodyPool.size)
        } else {
            val rb = RigidBody.upcast(body)
            //convert both active and kinematic objects (for their velocity)
            if (rb != null && (rb.inverseMass != 0f || rb.isKinematicObject)) {
                solverBodyIdA = tmpSolverBodyPool.size
                val solverBody = SolverBody()
                tmpSolverBodyPool += solverBody
                initSolverBody(solverBody, body, timeStep)
                body.companionId = solverBodyIdA
            } else {
                if (fixedBodyId < 0) {
                    fixedBodyId = tmpSolverBodyPool.size
                    val fixedBody = SolverBody()
                    tmpSolverBodyPool += fixedBody
                    initSolverBody(fixedBody, null, timeStep)
                }
                return fixedBodyId
//			return 0;//assume first one is a fixed solver body
            }
        }
        return solverBodyIdA
    }

    fun initSolverBody(solverBody: SolverBody, collisionObject: CollisionObject?, timeStep: Float) {

        val rb = collisionObject?.let { RigidBody.upcast(it) }

        solverBody.deltaLinearVelocity put 0f
        solverBody.deltaAngularVelocity put 0f
        solverBody.pushVelocity put 0f
        solverBody.turnVelocity put 0f

        if (rb != null) {
            solverBody.worldTransform put rb.getWorldTransform()
            (solverBody.invMass put rb.inverseMass) *= rb.linearFactor
            solverBody.originalBody = rb
            solverBody.angularFactor put rb.getAngularFactor()
            solverBody.linearFactor put rb.linearFactor
            solverBody.linearVelocity put rb._linearVelocity
            solverBody.angularVelocity put rb._angularVelocity
            solverBody.externalForceImpulse put rb.totalForce * rb.inverseMass * timeStep // TODO order
            solverBody.externalTorqueImpulse put rb.totalTorque * rb.invInertiaTensorWorld * timeStep
        } else {
            solverBody.worldTransform.setIdentity()
            solverBody.invMass put 0f
            solverBody.originalBody = null
            solverBody.angularFactor put 1f
            solverBody.linearFactor put 1f
            solverBody.linearVelocity put 0f
            solverBody.angularVelocity put 0f
            solverBody.externalForceImpulse put 0f
            solverBody.externalTorqueImpulse put 0f
        }
    }

    fun solveGroupCacheFriendlySplitImpulseIterations(bodies: ArrayList<CollisionObject>, numBodies: Int,
                                                      manifolds: ArrayList<PersistentManifold>, manifoldsPtr: Int, numManifolds: Int,
                                                      constraints: ArrayList<TypedConstraint>, constraintsPtr: Int, numConstraints: Int,
                                                      infoGlobal: ContactSolverInfo, debugDrawer: DebugDraw?) {

        if (infoGlobal.splitImpulse) {
            for (iteration in 0 until infoGlobal.numIterations) {
                var leastSquaresResidual = 0f
                run {
                    val numPoolConstraints = tmpSolverContactConstraintPool.size
                    for (j in 0 until numPoolConstraints) {
                        val solveManifold = tmpSolverContactConstraintPool[orderTmpConstraintPool[j]]
                        val residual = resolveSplitPenetrationImpulse(tmpSolverBodyPool[solveManifold.solverBodyIdA],
                                tmpSolverBodyPool[solveManifold.solverBodyIdB], solveManifold)
                        leastSquaresResidual += residual * residual
                    }
                }
//                if (leastSquaresResidual <= infoGlobal.leastSquaresResidualThreshold || iteration >= (infoGlobal.m_numIterations - 1)) { TODO
//                    #ifdef VERBOSE_RESIDUAL_PRINTF
//                            printf("residual = %f at iteration #%d\n", leastSquaresResidual, iteration)
//                    #endif
//                    break
//                }
            }
        }
    }

    fun solveGroupCacheFriendlyFinish(bodies: ArrayList<CollisionObject>, numBodies: Int, infoGlobal: ContactSolverInfo): Float {

        var numPoolConstraints = tmpSolverContactConstraintPool.size
        if (infoGlobal.solverMode has Sm.USE_WARMSTARTING) {
            for (j in 0 until numPoolConstraints) {
                val solveManifold = tmpSolverContactConstraintPool[j]
                val pt = solveManifold.originalContactPoint as ManifoldPoint
                pt.appliedImpulse = solveManifold.appliedImpulse
                //	float f = m_tmpSolverContactFrictionConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;
                //	printf("pt->m_appliedImpulseLateral1 = %f\n", f);
                pt.appliedImpulseLateral1 = tmpSolverContactFrictionConstraintPool[solveManifold.frictionIndex].appliedImpulse
                //printf("pt->m_appliedImpulseLateral1 = %f\n", pt->m_appliedImpulseLateral1);
                if (infoGlobal.solverMode has Sm.USE_2_FRICTION_DIRECTIONS)
                    pt.appliedImpulseLateral2 = tmpSolverContactFrictionConstraintPool[solveManifold.frictionIndex + 1].appliedImpulse
                //do a callback here?
            }
        }
        numPoolConstraints = tmpSolverNonContactConstraintPool.size
        for (j in 0 until numPoolConstraints) {
            val solverConstr = tmpSolverNonContactConstraintPool[j]
            val constr = solverConstr.originalContactPoint as TypedConstraint
            constr.jointFeedback?.let { fb ->
                fb.appliedForceBodyA += solverConstr.contactNormal1 * solverConstr.appliedImpulse * constr.rbA!!.linearFactor / infoGlobal.timeStep
                fb.appliedForceBodyB += solverConstr.contactNormal2 * solverConstr.appliedImpulse * constr.rbB!!.linearFactor / infoGlobal.timeStep
                fb.appliedTorqueBodyA += solverConstr.relpos1CrossNormal * constr.rbA!!.getAngularFactor() * solverConstr.appliedImpulse / infoGlobal.timeStep
                fb.appliedTorqueBodyB += solverConstr.relpos2CrossNormal * constr.rbB!!.getAngularFactor() * solverConstr.appliedImpulse / infoGlobal.timeStep /*RGM ???? */
            }
            constr.appliedImpulse = solverConstr.appliedImpulse
            if (abs(solverConstr.appliedImpulse) >= constr.breakingImpulseThreshold)
                constr.isEnabled = false
        }

        for (i in 0 until tmpSolverBodyPool.size) {
            tmpSolverBodyPool[i].originalBody?.let {
                if (infoGlobal.splitImpulse)
                    tmpSolverBodyPool[i].writebackVelocityAndTransform(infoGlobal.timeStep, infoGlobal.splitImpulseTurnErp)
                else
                    tmpSolverBodyPool[i].writebackVelocity()

                it.setLinearVelocity(tmpSolverBodyPool[i].linearVelocity + tmpSolverBodyPool[i].externalForceImpulse)
                it.setAngularVelocity(tmpSolverBodyPool[i].angularVelocity + tmpSolverBodyPool[i].externalTorqueImpulse)
                if (infoGlobal.splitImpulse)
                    it.setWorldTransform(tmpSolverBodyPool[i].worldTransform)
                tmpSolverBodyPool[i].originalBody!!.companionId = -1
            }
        }
        tmpSolverContactConstraintPool.clear()
        tmpSolverNonContactConstraintPool.clear()
        tmpSolverContactFrictionConstraintPool.clear()
        tmpSolverContactRollingFrictionConstraintPool.clear()
        tmpSolverBodyPool.clear()
        return 0f
    }

    fun solveSingleIteration(iteration: Int, bodies: ArrayList<CollisionObject>, numBodies: Int,
                             manifolds: ArrayList<PersistentManifold>, manifoldsPtr: Int, numManifolds: Int,
                             constraints: ArrayList<TypedConstraint>, constraintsPtr: Int, numConstraints: Int,
                             infoGlobal: ContactSolverInfo, debugDrawer: DebugDraw?): Float {

        var leastSquaresResidual = 0f

        val numNonContactPool = tmpSolverNonContactConstraintPool.size
        val numConstraintPool = tmpSolverContactConstraintPool.size
        val numFrictionPool = tmpSolverContactFrictionConstraintPool.size

        if (infoGlobal.solverMode has Sm.RANDOMIZE_ORDER) {

            for (j in 0 until numNonContactPool) {
                val tmp = orderNonContactConstraintPool[j]
                val swapi = randInt2(j + 1)
                orderNonContactConstraintPool[j] = orderNonContactConstraintPool[swapi]
                orderNonContactConstraintPool[swapi] = tmp
            }

            //contact/friction constraints are not solved more than
            if (iteration < infoGlobal.numIterations) {
                for (j in 0 until numConstraintPool) {
                    val tmp = orderTmpConstraintPool[j]
                    val swapi = randInt2(j + 1)
                    orderTmpConstraintPool[j] = orderTmpConstraintPool[swapi]
                    orderTmpConstraintPool[swapi] = tmp
                }
                for (j in 0 until numFrictionPool) {
                    val tmp = orderFrictionConstraintPool[j]
                    val swapi = randInt2(j + 1)
                    orderFrictionConstraintPool[j] = orderFrictionConstraintPool[swapi]
                    orderFrictionConstraintPool[swapi] = tmp
                }
            }
        }

        ///solve all joint constraints
        for (j in 0 until tmpSolverNonContactConstraintPool.size) {
            val constraint = tmpSolverNonContactConstraintPool[orderNonContactConstraintPool[j]]
            if (iteration < constraint.overrideNumSolverIterations) {
                val residual = resolveSingleConstraintRowGeneric!!(tmpSolverBodyPool[constraint.solverBodyIdA],
                        tmpSolverBodyPool[constraint.solverBodyIdB], constraint)
                leastSquaresResidual += residual * residual
            }
        }

        if (iteration < infoGlobal.numIterations) {
            for (j in 0 until numConstraints)
                if (constraints[constraintsPtr + j].isEnabled) {
                    val bodyAid = getOrInitSolverBody(constraints[constraintsPtr + j].rbA!!, infoGlobal.timeStep)
                    val bodyBid = getOrInitSolverBody(constraints[constraintsPtr + j].rbB!!, infoGlobal.timeStep)
                    val bodyA = tmpSolverBodyPool[bodyAid]
                    val bodyB = tmpSolverBodyPool[bodyBid]
                    constraints[constraintsPtr + j].solveConstraintObsolete(bodyA, bodyB, infoGlobal.timeStep)
                }

            // solve all contact constraints
            if (infoGlobal.solverMode has Sm.INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS) {

                val numPoolConstraints = tmpSolverContactConstraintPool.size
                val multiplier = if (infoGlobal.solverMode has Sm.USE_2_FRICTION_DIRECTIONS) 2 else 1

                for (c in 0 until numPoolConstraints) {

                    var totalImpulse = 0f
                    run {
                        val solveManifold = tmpSolverContactConstraintPool[orderTmpConstraintPool[c]]
                        val residual = resolveSingleConstraintRowLowerLimit!!(tmpSolverBodyPool[solveManifold.solverBodyIdA],
                                tmpSolverBodyPool[solveManifold.solverBodyIdB], solveManifold)
                        leastSquaresResidual += residual * residual

                        totalImpulse = solveManifold.appliedImpulse
                    }
                    val applyFriction = true
                    if (applyFriction) {
                        run {
                            val solveManifold = tmpSolverContactFrictionConstraintPool[orderFrictionConstraintPool[c * multiplier]]

                            if (totalImpulse > 0f) {
                                solveManifold.lowerLimit = -(solveManifold.friction * totalImpulse)
                                solveManifold.upperLimit = solveManifold.friction * totalImpulse

                                val residual = resolveSingleConstraintRowGeneric!!(tmpSolverBodyPool[solveManifold.solverBodyIdA],
                                        tmpSolverBodyPool[solveManifold.solverBodyIdB], solveManifold)
                                leastSquaresResidual += residual * residual
                            }
                        }

                        if (infoGlobal.solverMode has Sm.USE_2_FRICTION_DIRECTIONS) {

                            val solveManifold = tmpSolverContactFrictionConstraintPool[orderFrictionConstraintPool[c * multiplier + 1]]

                            if (totalImpulse > 0f) {
                                solveManifold.lowerLimit = -(solveManifold.friction * totalImpulse)
                                solveManifold.upperLimit = solveManifold.friction * totalImpulse

                                val residual = resolveSingleConstraintRowGeneric!!(tmpSolverBodyPool[solveManifold.solverBodyIdA],
                                        tmpSolverBodyPool[solveManifold.solverBodyIdB], solveManifold)
                                leastSquaresResidual += residual * residual
                            }
                        }
                    }
                }

            } else {    // SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS
                // solve the friction constraints after all contact constraints, don't interleave them
                val numPoolConstraints = tmpSolverContactConstraintPool.size

                for (j in 0 until numPoolConstraints) {
                    val solveManifold = tmpSolverContactConstraintPool[orderTmpConstraintPool[j]]
                    val residual = resolveSingleConstraintRowLowerLimit!!(tmpSolverBodyPool[solveManifold.solverBodyIdA],
                            tmpSolverBodyPool[solveManifold.solverBodyIdB], solveManifold)
                    leastSquaresResidual += residual * residual
                }

                // solve all friction constraints
                val numFrictionPoolConstraints = tmpSolverContactFrictionConstraintPool.size
                for (j in 0 until numFrictionPoolConstraints) {
                    val solveManifold = tmpSolverContactFrictionConstraintPool[orderFrictionConstraintPool[j]]
                    val totalImpulse = tmpSolverContactConstraintPool[solveManifold.frictionIndex].appliedImpulse

                    if (totalImpulse > 0f) {
                        solveManifold.lowerLimit = -(solveManifold.friction * totalImpulse)
                        solveManifold.upperLimit = solveManifold.friction * totalImpulse

                        val residual = resolveSingleConstraintRowGeneric!!(tmpSolverBodyPool[solveManifold.solverBodyIdA],
                                tmpSolverBodyPool[solveManifold.solverBodyIdB], solveManifold)
                        leastSquaresResidual += residual * residual
                    }
                }
            }

            val numRollingFrictionPoolConstraints = tmpSolverContactRollingFrictionConstraintPool.size
            for (j in 0 until numRollingFrictionPoolConstraints) {
                val rollingFrictionConstraint = tmpSolverContactRollingFrictionConstraintPool[j]
                val totalImpulse = tmpSolverContactConstraintPool[rollingFrictionConstraint.frictionIndex].appliedImpulse
                if (totalImpulse > 0f) {
                    var rollingFrictionMagnitude = rollingFrictionConstraint.friction * totalImpulse
                    if (rollingFrictionMagnitude > rollingFrictionConstraint.friction)
                        rollingFrictionMagnitude = rollingFrictionConstraint.friction

                    rollingFrictionConstraint.lowerLimit = -rollingFrictionMagnitude
                    rollingFrictionConstraint.upperLimit = rollingFrictionMagnitude

                    val residual = resolveSingleConstraintRowGeneric!!(tmpSolverBodyPool[rollingFrictionConstraint.solverBodyIdA],
                            tmpSolverBodyPool[rollingFrictionConstraint.solverBodyIdB], rollingFrictionConstraint)
                    leastSquaresResidual += residual * residual
                }
            }
        }
        return leastSquaresResidual
    }

    fun solveGroupCacheFriendlySetup(bodies: ArrayList<CollisionObject>, numBodies: Int,
                                     manifolds: ArrayList<PersistentManifold>, manifoldsPtr: Int, numManifolds: Int,
                                     constraints: ArrayList<TypedConstraint>, constraintsPtr: Int, numConstraints: Int,
                                     infoGlobal: ContactSolverInfo, debugDrawer: DebugDraw?): Float {

        fixedBodyId = -1
        BT_PROFILE("solveGroupCacheFriendlySetup")

        // if solver mode has changed,
        if (infoGlobal.solverMode != cachedSolverMode) {
            // update solver functions to use SIMD or non-SIMD
            val useSimd = infoGlobal.solverMode has Sm.SIMD
            setupSolverFunctions(useSimd)
            cachedSolverMode = infoGlobal.solverMode
        }
        maxOverrideNumSolverIterations = 0

        for (i in 0 until numBodies)
            bodies[i].companionId = -1

        tmpSolverBodyPool.clear()

        //btSolverBody& fixedBody = m_tmpSolverBodyPool.expand();
        //initSolverBody(&fixedBody,0);

        //convert all bodies
        for (i in 0 until numBodies) {

            val bodyId = getOrInitSolverBody(bodies[i], infoGlobal.timeStep)

            val body = RigidBody.upcast(bodies[i])
            if (body != null && body.inverseMass != 0f) {
                val solverBody = tmpSolverBodyPool[bodyId]
                val gyroForce = Vec3()
                if (body.rigidbodyFlags has Rbf.ENABLE_GYROSCOPIC_FORCE_EXPLICIT) {
                    gyroForce put body.computeGyroscopicForceExplicit(infoGlobal.maxGyroscopicForce)
                    solverBody.externalTorqueImpulse -= gyroForce * body.invInertiaTensorWorld * infoGlobal.timeStep
                }
                if (body.rigidbodyFlags has Rbf.ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD) {
                    gyroForce put body.computeGyroscopicImpulseImplicit_World(infoGlobal.timeStep)
                    solverBody.externalTorqueImpulse += gyroForce
                }
                if (body.rigidbodyFlags has Rbf.ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY) {
                    gyroForce put body.computeGyroscopicImpulseImplicit_Body(infoGlobal.timeStep)
                    solverBody.externalTorqueImpulse += gyroForce
                }
            }
        }

        for (j in 0 until numConstraints) {
            val constraint = constraints[constraintsPtr + j]
            constraint.buildJacobian()
            constraint.appliedImpulse = 0f
        }

        //btRigidBody* rb0=0,*rb1=0;

        var totalNumRows = 0

        tmpConstraintSizesPool resize numConstraints
        //calculate the total number of contraint rows
        for (i in 0 until numConstraints) {
            val info1 = tmpConstraintSizesPool[i]
            constraints[constraintsPtr + i].jointFeedback?.apply {
                appliedForceBodyA put 0f
                appliedTorqueBodyA put 0f
                appliedForceBodyB put 0f
                appliedTorqueBodyB put 0f
            }
            if (constraints[constraintsPtr + i].isEnabled)
                constraints[constraintsPtr + i].getInfo1(info1)
            else {
                info1.numConstraintRows = 0
                info1.nub = 0
            }
            totalNumRows += info1.numConstraintRows
        }
        tmpSolverNonContactConstraintPool resize totalNumRows

        // setup the btSolverConstraints
        var currentRow = 0

        for (i in 0 until numConstraints) {

            val info1 = tmpConstraintSizesPool[i]

            if (info1.numConstraintRows != 0) {

                assert(currentRow < totalNumRows)

                val constraint = constraints[constraintsPtr + i]
                val rbA = constraint.rbA
                val rbB = constraint.rbB

                val solverBodyIdA = getOrInitSolverBody(rbA!!, infoGlobal.timeStep)
                val solverBodyIdB = getOrInitSolverBody(rbB!!, infoGlobal.timeStep)

                val bodyAPtr = tmpSolverBodyPool[solverBodyIdA]
                val bodyBPtr = tmpSolverBodyPool[solverBodyIdB]

                val overrideNumSolverIterations = if (constraint.overrideNumSolverIterations > 0)
                    constraint.overrideNumSolverIterations else infoGlobal.numIterations
                if (overrideNumSolverIterations > maxOverrideNumSolverIterations)
                    maxOverrideNumSolverIterations = overrideNumSolverIterations

                for (j in 0 until info1.numConstraintRows) with(tmpSolverNonContactConstraintPool[currentRow + j]) {
                    lowerLimit = -Float.MAX_VALUE
                    upperLimit = Float.MAX_VALUE
                    appliedImpulse = 0f
                    appliedPushImpulse = 0f
                    this.solverBodyIdA = solverBodyIdA
                    this.solverBodyIdB = solverBodyIdB
                    this.overrideNumSolverIterations = overrideNumSolverIterations
                }
                with(bodyAPtr) {
                    deltaLinearVelocity put 0f
                    deltaAngularVelocity put 0f
                    pushVelocity put 0f
                    turnVelocity put 0f
                }
                with(bodyBPtr) {
                    deltaLinearVelocity put 0f
                    deltaAngularVelocity put 0f
                    pushVelocity put 0f
                    turnVelocity put 0f
                }

                val info2 = TypedConstraint.ConstraintInfo2().apply {
                    fps = 1f / infoGlobal.timeStep
                    erp = infoGlobal.erp
                    val currentConstraintRow = tmpSolverNonContactConstraintPool[currentRow]
                    j1linearAxis = currentConstraintRow.contactNormal1
                    j1angularAxis = currentConstraintRow.relpos1CrossNormal
                    j2linearAxis = currentConstraintRow.contactNormal2
                    j2angularAxis = currentConstraintRow.relpos2CrossNormal
                    rowSkip = SolverConstraint.size / Float.BYTES   //check this
                    ///the size of btSolverConstraint needs be a multiple of btScalar
                    assert(rowSkip * Float.BYTES == SolverConstraint.size)
                    constraintError = currentConstraintRow.rhs
                    currentConstraintRow.cfm = infoGlobal.globalCfm
                    damping = infoGlobal.damping
                    cfm = currentConstraintRow.cfm
                    lowerLimit = currentConstraintRow.lowerLimit
                    upperLimit = currentConstraintRow.upperLimit
                    numIterations = infoGlobal.numIterations
                }
                constraints[constraintsPtr + i] getInfo2 info2

                // finalize the constraint setup
                for (j in 0 until info1.numConstraintRows) {

                    val solverConstraint = tmpSolverNonContactConstraintPool[currentRow + j]

                    if (solverConstraint.upperLimit >= constraints[constraintsPtr + i].breakingImpulseThreshold)
                        solverConstraint.upperLimit = constraints[constraintsPtr + i].breakingImpulseThreshold

                    if (solverConstraint.lowerLimit <= -constraints[constraintsPtr + i].breakingImpulseThreshold)
                        solverConstraint.lowerLimit = -constraints[constraintsPtr + i].breakingImpulseThreshold

                    solverConstraint.originalContactPoint = constraint

                    val ftorqueAxis1 = solverConstraint.relpos1CrossNormal
                    solverConstraint.angularComponentA put constraint.rbA!!.invInertiaTensorWorld * ftorqueAxis1 * constraint.rbA!!.getAngularFactor()

                    val ftorqueAxis2 = solverConstraint.relpos2CrossNormal
                    solverConstraint.angularComponentB put constraint.rbB!!.invInertiaTensorWorld * ftorqueAxis2 * constraint.rbB!!.getAngularFactor()

                    val iMJlA = solverConstraint.contactNormal1 * rbA.inverseMass
                    val iMJaA = rbA.invInertiaTensorWorld * solverConstraint.relpos1CrossNormal
                    val iMJlB = solverConstraint.contactNormal2 * rbB.inverseMass//sign of normal?
                    val iMJaB = rbB.invInertiaTensorWorld * solverConstraint.relpos2CrossNormal

                    var sum = iMJlA dot solverConstraint.contactNormal1
                    sum += iMJaA dot solverConstraint.relpos1CrossNormal
                    sum += iMJlB dot solverConstraint.contactNormal2
                    sum += iMJaB dot solverConstraint.relpos2CrossNormal
                    val fsum = abs(sum)
                    assert(fsum > Float.EPSILON)
                    val sorRelaxation = 1f //todo: get from globalInfo?
                    solverConstraint.jacDiagABInv = if (fsum > Float.EPSILON) sorRelaxation / sum else 0f

                    val externalForceImpulseA = if (bodyAPtr.originalBody != null) bodyAPtr.externalForceImpulse else Vec3()
                    val externalTorqueImpulseA = if (bodyAPtr.originalBody != null) bodyAPtr.externalTorqueImpulse else Vec3()

                    val externalForceImpulseB = if (bodyBPtr.originalBody != null) bodyBPtr.externalForceImpulse else Vec3()
                    val externalTorqueImpulseB = if (bodyBPtr.originalBody != null) bodyBPtr.externalTorqueImpulse else Vec3()

                    val vel1Dotn = solverConstraint.contactNormal1.dot(rbA._linearVelocity + externalForceImpulseA) +
                            solverConstraint.relpos1CrossNormal.dot(rbA._angularVelocity + externalTorqueImpulseA)

                    val vel2Dotn = solverConstraint.contactNormal2.dot(rbB._linearVelocity + externalForceImpulseB) +
                            solverConstraint.relpos2CrossNormal.dot(rbB._angularVelocity + externalTorqueImpulseB)

                    val relVel = vel1Dotn + vel2Dotn
                    val restitution = 0f
                    val positionalError = solverConstraint.rhs // already filled in by getConstraintInfo2
                    val velocityError = restitution - relVel * info2.damping
                    val penetrationImpulse = positionalError * solverConstraint.jacDiagABInv
                    val velocityImpulse = velocityError * solverConstraint.jacDiagABInv
                    solverConstraint.rhs = penetrationImpulse + velocityImpulse
                    solverConstraint.appliedImpulse = 0f
                }
            }
            currentRow += tmpConstraintSizesPool[i].numConstraintRows
        }
        convertContacts(manifolds, manifoldsPtr, numManifolds, infoGlobal)

//	btContactSolverInfo info = infoGlobal;

        val numNonContactPool = tmpSolverNonContactConstraintPool.size
        val numConstraintPool = tmpSolverContactConstraintPool.size
        val numFrictionPool = tmpSolverContactFrictionConstraintPool.size

        ///@todo: use stack allocator for such temporarily memory, same for solver bodies/constraints
        resize(::orderNonContactConstraintPool, numNonContactPool)
        val newSize = numConstraintPool * if (infoGlobal.solverMode has Sm.USE_2_FRICTION_DIRECTIONS) 2 else 1
        resize(::orderTmpConstraintPool, newSize)

        resize(::orderFrictionConstraintPool, numFrictionPool)
        for (i in 0 until numNonContactPool) orderNonContactConstraintPool[i] = i
        for (i in 0 until numConstraintPool) orderTmpConstraintPool[i] = i
        for (i in 0 until numFrictionPool) orderFrictionConstraintPool[i] = i
        return 0f
    }

    fun solveGroupCacheFriendlyIterations(bodies: ArrayList<CollisionObject>, numBodies: Int,
                                          manifolds: ArrayList<PersistentManifold>, manifoldsPtr: Int, numManifolds: Int,
                                          constraints: ArrayList<TypedConstraint>, constraintsPtr: Int, numConstraints: Int,
                                          infoGlobal: ContactSolverInfo, debugDrawer: DebugDraw?): Float {

        // this is a special step to resolve penetrations (just for contacts)
        solveGroupCacheFriendlySplitImpulseIterations(bodies, numBodies, manifolds, manifoldsPtr, numManifolds, constraints,
                constraintsPtr, numConstraints, infoGlobal, debugDrawer)

        val maxIterations = if (maxOverrideNumSolverIterations > infoGlobal.numIterations) maxOverrideNumSolverIterations else infoGlobal.numIterations

        for (iteration in 0 until maxIterations) {  //for ( int iteration = maxIterations-1  ; iteration >= 0;iteration--)
            val leastSquaresResidual = solveSingleIteration(iteration, bodies, numBodies, manifolds, manifoldsPtr, numManifolds,
                    constraints, constraintsPtr, numConstraints, infoGlobal, debugDrawer)
//            if (leastSquaresResidual <= infoGlobal.leastSquaresResidualThreshold || iteration >= (maxIterations - 1)) { TODO
//                #ifdef VERBOSE_RESIDUAL_PRINTF
//                        printf("residual = %f at iteration #%d\n", m_leastSquaresResidual, iteration)
//                #endif
//                break
//            }
        }
        return 0f
    }


    override fun solveGroup(bodies: ArrayList<CollisionObject>, numBodies: Int,
                            manifolds: ArrayList<PersistentManifold>, manifoldsPtr: Int, numManifolds: Int,
                            constraints: ArrayList<TypedConstraint>, constraintsPtr: Int, numConstraints: Int,
                            infoGlobal: ContactSolverInfo, debugDrawer: DebugDraw?, dispatcher: Dispatcher): Float {

        BT_PROFILE("solveGroup")
        //you need to provide at least some bodies

        solveGroupCacheFriendlySetup(bodies, numBodies, manifolds, manifoldsPtr, numManifolds, constraints, constraintsPtr,
                numConstraints, infoGlobal, debugDrawer)

        solveGroupCacheFriendlyIterations(bodies, numBodies, manifolds, manifoldsPtr, numManifolds, constraints, constraintsPtr,
                numConstraints, infoGlobal, debugDrawer)

        solveGroupCacheFriendlyFinish(bodies, numBodies, infoGlobal)

        return 0f
    }


    override fun reset() {
        seed2 = 0
    }

    fun rand2() = (1664525L * seed2 + 1013904223L) and 0xffffffff

    /** See ODE: adam's all-int straightforward(?) dRandInt (0..n-1)    */
    fun randInt2(n: Int): Int {

        // seems good; xor-fold and modulus
        val un = n.L
        var r = rand2()

        // note: probably more aggressive than it needs to be -- might be able to get away without one or two of the innermost branches.
        if (un <= 0x00010000) {
            r = r xor (r ushr 16)
            if (un <= 0x00000100) {
                r = r xor (r ushr 8)
                if (un <= 0x00000010) {
                    r = r xor (r ushr 4)
                    if (un <= 0x00000004) {
                        r = r xor (r ushr 2)
                        if (un <= 0x00000002)
                            r = r xor (r ushr 1)
                    }
                }
            }
        }
        return (r % un).toInt()
    }

//
//    void    setRandSeed(unsigned long seed)
//    {
//        m_btSeed2 = seed
//    }
//    unsigned long    getRandSeed()
//    const
//    {
//        return m_btSeed2
//    }
//
//
//    virtual btConstraintSolverType    getSolverType()
//    const
//    {
//        return BT_SEQUENTIAL_IMPULSE_SOLVER
//    }
//
//    btSingleConstraintRowSolver    getActiveConstraintRowSolverGeneric()
//    {
//        return m_resolveSingleConstraintRowGeneric
//    }
//    void setConstraintRowSolverGeneric(btSingleConstraintRowSolver rowSolver)
//    {
//        m_resolveSingleConstraintRowGeneric = rowSolver
//    }
//    btSingleConstraintRowSolver    getActiveConstraintRowSolverLowerLimit()
//    {
//        return m_resolveSingleConstraintRowLowerLimit
//    }
//    void setConstraintRowSolverLowerLimit(btSingleConstraintRowSolver rowSolver)
//    {
//        m_resolveSingleConstraintRowLowerLimit = rowSolver
//    }
//
//    ///Various implementations of solving a single constraint row using a generic equality constraint, using scalar reference, SSE2 or SSE4
//    btSingleConstraintRowSolver    getScalarConstraintRowSolverGeneric()
//    btSingleConstraintRowSolver    getSSE2ConstraintRowSolverGeneric()
//    btSingleConstraintRowSolver    getSSE4_1ConstraintRowSolverGeneric()
//
//    ///Various implementations of solving a single constraint row using an inequality (lower limit) constraint, using scalar reference, SSE2 or SSE4
//    btSingleConstraintRowSolver    getScalarConstraintRowSolverLowerLimit()
//    btSingleConstraintRowSolver    getSSE2ConstraintRowSolverLowerLimit()
//    btSingleConstraintRowSolver    getSSE4_1ConstraintRowSolverLowerLimit()

    companion object {
        /** This is the scalar reference implementation of solving a single constraint row, the innerloop of the
         *  Projected Gauss Seidel/Sequential Impulse constraint solver */
        fun resolveSingleConstraintRowGeneric(body1: SolverBody, body2: SolverBody, c: SolverConstraint): Float {
            var deltaImpulse = c.rhs - c.appliedImpulse * c.cfm
            val deltaVel1Dotn = c.contactNormal1.dot(body1.deltaLinearVelocity) + c.relpos1CrossNormal.dot(body1.deltaAngularVelocity)
            val deltaVel2Dotn = c.contactNormal2.dot(body2.deltaLinearVelocity) + c.relpos2CrossNormal.dot(body2.deltaAngularVelocity)

//	const btScalar delta_rel_vel	=	deltaVel1Dotn-deltaVel2Dotn;
            deltaImpulse -= deltaVel1Dotn * c.jacDiagABInv
            deltaImpulse -= deltaVel2Dotn * c.jacDiagABInv

            val sum = c.appliedImpulse + deltaImpulse
            c.appliedImpulse = when {
                sum < c.lowerLimit -> {
                    deltaImpulse = c.lowerLimit - c.appliedImpulse
                    c.lowerLimit
                }
                sum > c.upperLimit -> {
                    deltaImpulse = c.upperLimit - c.appliedImpulse
                    c.upperLimit
                }
                else -> sum
            }
            body1.internalApplyImpulse(c.contactNormal1 * body1.invMass, c.angularComponentA, deltaImpulse)
            body2.internalApplyImpulse(c.contactNormal2 * body2.invMass, c.angularComponentB, deltaImpulse)

            return deltaImpulse
        }

        fun resolveSingleConstraintRowLowerLimit(body1: SolverBody, body2: SolverBody, c: SolverConstraint): Float {
            var deltaImpulse = c.rhs - c.appliedImpulse * c.cfm
            val deltaVel1Dotn = c.contactNormal1.dot(body1.deltaLinearVelocity) + c.relpos1CrossNormal.dot(body1.deltaAngularVelocity)
            val deltaVel2Dotn = c.contactNormal2.dot(body2.deltaLinearVelocity) + c.relpos2CrossNormal.dot(body2.deltaAngularVelocity)

            deltaImpulse -= deltaVel1Dotn * c.jacDiagABInv
            deltaImpulse -= deltaVel2Dotn * c.jacDiagABInv
            val sum = c.appliedImpulse + deltaImpulse
            c.appliedImpulse = if (sum < c.lowerLimit) {
                deltaImpulse = c.lowerLimit - c.appliedImpulse
                c.lowerLimit
            } else sum
            body1.internalApplyImpulse(c.contactNormal1 * body1.invMass, c.angularComponentA, deltaImpulse)
            body2.internalApplyImpulse(c.contactNormal2 * body2.invMass, c.angularComponentB, deltaImpulse)

            return deltaImpulse
        }

        fun resolveSplitPenetrationImpulse(body1: SolverBody, body2: SolverBody, c: SolverConstraint): Float {
            var deltaImpulse = 0f

            if (c.rhsPenetration != 0f) {
                numSplitImpulseRecoveries++
                deltaImpulse = c.rhsPenetration - c.appliedPushImpulse * c.cfm
                val deltaVel1Dotn = c.contactNormal1.dot(body1.pushVelocity) + c.relpos1CrossNormal.dot(body1.turnVelocity)
                val deltaVel2Dotn = c.contactNormal2.dot(body2.pushVelocity) + c.relpos2CrossNormal.dot(body2.turnVelocity)

                deltaImpulse -= deltaVel1Dotn * c.jacDiagABInv
                deltaImpulse -= deltaVel2Dotn * c.jacDiagABInv
                val sum = c.appliedPushImpulse + deltaImpulse
                c.appliedPushImpulse = if (sum < c.lowerLimit) {
                    deltaImpulse = c.lowerLimit - c.appliedPushImpulse
                    c.lowerLimit
                } else sum
                body1.internalApplyPushImpulse(c.contactNormal1 * body1.invMass, c.angularComponentA, deltaImpulse)
                body2.internalApplyPushImpulse(c.contactNormal2 * body2.invMass, c.angularComponentB, deltaImpulse)
            }
            return deltaImpulse
        }

        fun resize(intArray: KMutableProperty0<IntArray>, newSize: Int) {
            val array = intArray()
            val size = array.size
            if (size != newSize) {
                val newArray = if (size < newSize) IntArray(newSize) { array.getOrElse(it) { 0 } } else IntArray(newSize) { array[it] }
                intArray.set(newArray)
            }
        }
    }
}
