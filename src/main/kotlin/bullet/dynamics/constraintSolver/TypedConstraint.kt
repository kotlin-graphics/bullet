/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2010 Erwin Coumans  http://continuousphysics.com/Bullet/

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

import bullet.dynamics.dynamics.RigidBody
import bullet.linearMath.*
import kotlin.math.abs

/** Don't change any of the existing enum values, so add enum types at the end for serialization compatibility  */
enum class TypedConstraintType {
    POINT2POINT,
    HINGE,
    CONETWIST,
    D6,
    SLIDER,
    CONTACT,
    D6_SPRING,
    GEAR,
    FIXED,
    D6_SPRING_2,
    MAX;

    val i = ordinal + 3

    companion object {
        infix fun of(i: Int) = values().first { it.i == i }
    }
}


enum class ConstraintParams {
    ERP,
    STOP_ERP,
    CFM,
    STOP_CFM;

    val i = ordinal + 1
}


class JointFeedback {
    val appliedForceBodyA = Vec3()
    val appliedTorqueBodyA = Vec3()
    val appliedForceBodyB = Vec3()
    val appliedTorqueBodyB = Vec3()
}

val DEFAULT_DEBUGDRAW_SIZE = 0.05f

/** TypedConstraint is the baseclass for Bullet constraints and vehicles    */
abstract class TypedConstraint(type: TypedConstraintType, var rbA: RigidBody?, var rbB: RigidBody? = fixedBody) : TypedObject(type.i) {

    var userConstraintType = -1

    private var union: Any? = -1
    var userConstraintId
        get() = union as Int
        set(value) {
            union = value
        }
    var userConstraintPtr // TODO check
        get() = union as Int
        set(value) {
            union = value
        }

    var breakingImpulseThreshold = Float.MAX_VALUE
    var isEnabled = true
    var needsFeedback = false
    var overrideNumSolverIterations = -1

    var appliedImpulse = 0f
        /** getAppliedImpulse is an estimated total applied impulse.
         *  This feedback could be used to determine breaking constraints or playing sounds.    */
        get() {
            assert(needsFeedback)
            return field
        }
    var dbgDrawSize = DEFAULT_DEBUGDRAW_SIZE
    var jointFeedback: JointFeedback? = null

    /** internal method used by the constraint solver, don't use them directly  */
    fun getMotorFactor(pos: Float, lowLim: Float, uppLim: Float, vel: Float, timeFact: Float): Float {
        if (lowLim > uppLim) return 1f
        else if (lowLim == uppLim) return 0f
        var limFact = 1f
        val deltaMax = vel / timeFact
        return when {
            deltaMax < 0f -> when {
                pos >= lowLim && pos < (lowLim - deltaMax) -> (lowLim - pos) / deltaMax
                pos < lowLim -> 0f
                else -> 1f
            }
            deltaMax > 0f -> when {
                pos <= uppLim && pos > (uppLim - deltaMax) -> (uppLim - pos) / deltaMax
                pos > uppLim -> 0f
                else -> 1f
            }
            else -> 0f
        }
    }

    class ConstraintInfo1 {
        var numConstraintRows = 0
        var nub = 0
    }

    companion object {
        val fixed = RigidBody(0f, null, null)
        val fixedBody get() = fixed.also { it.setMassProps(0f, Vec3()) }
    }


    class ConstraintInfo2 {
        /** integrator parameter, frames per second (1/stepsize) */
        var fps = 0f
        /** integrator parameter, default error reduction parameter (0..1). */
        var erp = 0f
        /*  For the first and second body, pointers to two (linear and angular) n*3 jacobian sub matrices,
            stored by rows. These matrices will have been initialized to 0 on entry. if the second body is zero then
            the J2xx pointers may be 0.    */
        lateinit var j1linearAxis: Vec3
        lateinit var j1angularAxis: Vec3
        lateinit var j2linearAxis: Vec3
        lateinit var j2angularAxis: Vec3
        /** elements to jump from one row to the next in J's    */
        var rowSkip = 0
        /*  right hand sides of the equation J*v = c + cfm * lambda. cfm is the "constraint force mixing" vector.
            c is set to zero on entry, cfm is set to a constant value (typically very small or zero) value on entry.    */
        var constraintError = 0f
        var cfm = 0f
        // lo and hi limits for variables (set to -/+ infinity on entry).
        var lowerLimit = 0f
        var upperLimit = 0f
        /** number of solver iterations */
        var numIterations = 0
        /** damping of the velocity */
        var damping = 0f
    }

    /** internal method used by the constraint solver, don't use them directly  */
    abstract fun buildJacobian()

    /** internal method used by the constraint solver, don't use them directly  */
    abstract fun setupSolverConstraint(ca: ArrayList<SolverConstraint>, solverBodyA: Int, solverBodyB: Int, timeStep: Float)

    /** internal method used by the constraint solver, don't use them directly  */
    abstract fun getInfo1(info: ConstraintInfo1)

    /** internal method used by the constraint solver, don't use them directly  */
    abstract infix fun getInfo2(info: ConstraintInfo2)

    /** internal method used by the constraint solver, don't use them directly  */
    abstract fun solveConstraintObsolete(bodyA: SolverBody, bodyB: SolverBody, timeStep: Float)

    val constraintType get() = TypedConstraintType of objectType

    /** override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5).
     *  If no axis is provided, it uses the default axis for this constraint.   */
    abstract fun setParam(num: Int, value: Float, axis: Int = -1)

    /** return the local value of parameter */
    abstract fun getParam(num: Float, axis: Int = -1): Float

    abstract fun calculateSerializeBufferSize(): Int
}

/** @returns angle in range [-SIMD_2_PI, SIMD_2_PI], closest to one of the limits
 *  all arguments should be normalized angles (i.e. in range [-SIMD_PI, SIMD_PI])   */
fun adjustAngleToLimits(angleInRadians: Float, angleLowerLimitInRadians: Float, angleUpperLimitInRadians: Float) = when {
    angleLowerLimitInRadians >= angleUpperLimitInRadians -> angleInRadians
    angleInRadians < angleLowerLimitInRadians -> {
        val diffLo = abs(normalizeAngle(angleLowerLimitInRadians - angleInRadians))
        val diffHi = abs(normalizeAngle(angleUpperLimitInRadians - angleInRadians))
        if (diffLo < diffHi) angleInRadians else angleInRadians + PI2
    }
    angleInRadians > angleUpperLimitInRadians -> {
        val diffHi = abs(normalizeAngle(angleInRadians - angleUpperLimitInRadians))
        val diffLo = abs(normalizeAngle(angleInRadians - angleLowerLimitInRadians))
        if (diffLo < diffHi) angleInRadians - PI2 else angleInRadians
    }
    else -> angleInRadians
}

class AngularLimit {
    var center = 0f
    var halfRange = -1f
    var softness = 0.9f
    var biasFactor = 0.3f
    var relaxationFactor = 1f
    var correction = 0f
    var sign = 0f

    var solveLimit = false

    /** Sets all limit's parameters.
     *  When low > high limit becomes inactive.
     *  When high - low > 2PI limit is ineffective too becouse no angle can exceed the limit    */
    fun set(low: Float, high: Float, softness: Float = 0.9f, biasFactor: Float = 0.3f, relaxationFactor: Float = 1f) {
        halfRange = (high - low) / 2f
        center = normalizeAngle(low + halfRange)
        this.softness = softness
        this.biasFactor = biasFactor
        this.relaxationFactor = relaxationFactor
    }

    /** Checks conastaint angle against limit. If limit is active and the angle violates the limit correction is calculated.    */
    fun test(angle: Float) {
        correction = 0f
        sign = 0f
        solveLimit = false

        if (halfRange >= 0f) {
            val deviation = normalizeAngle(angle - center)
            if (deviation < -halfRange) {
                solveLimit = true
                correction = -(deviation + halfRange)
                sign = +1f
            } else if (deviation > halfRange) {
                solveLimit = true
                correction = halfRange - deviation
                sign = -1f
            }
        }
    }

    /** Checks given angle against limit. If limit is active and angle doesn't fit it, the angle returned is modified
     *  so it equals to the limit closest to given angle.
     *  JVM specific, angle in return   */
    fun fit(angle: Float): Float {
        if (halfRange > 0f) {
            val relativeAngle = normalizeAngle(angle - center)
            if (!equal(relativeAngle, halfRange))
                return if (relativeAngle > 0f) high else low
        }
        return angle
    }

    /** Returns correction value multiplied by sign value   */
    val error get() = correction * sign

    val low get() = normalizeAngle(center - halfRange)

    val high get() = normalizeAngle(center + halfRange)

}