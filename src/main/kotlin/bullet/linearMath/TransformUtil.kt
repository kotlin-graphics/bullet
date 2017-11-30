/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

package bullet.linearMath

import bullet.EPSILON
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

val ANGULAR_MOTION_THRESHOLD = 0.5f * HALF_PI

fun AabbSupport(halfExtents: Vec3, supportDir: Vec3) = Vec3(
        if (supportDir.x < 0f) -halfExtents.x else halfExtents.x,
        if (supportDir.y < 0f) -halfExtents.y else halfExtents.y,
        if (supportDir.z < 0f) -halfExtents.z else halfExtents.z)

/** Utils related to temporal transforms    */
object TransformUtil {

    fun integrateTransform(curTrans: Transform, linVel: Vec3, angVel: Vec3, timeStep: Float, predictedTransform: Transform) {

        predictedTransform.origin = curTrans.origin + linVel * timeStep

        val angle2 = angVel.length2()
        var angle = 0f
        if (angle2 > Float.EPSILON) angle = sqrt(angle2)

        //limit the angular motion
        if (angle * timeStep > ANGULAR_MOTION_THRESHOLD)
            angle = ANGULAR_MOTION_THRESHOLD / timeStep

        val axis = if (angle < 0.001f)  // use Taylor's expansions of sync function
            angVel * (0.5f * timeStep - (timeStep * timeStep * timeStep) * 0.020833333333f * angle * angle)
        else    // sync(angle) = sin(c * angle) / t
            angVel * (sin(0.5f * angle * timeStep) / angle)
        val dorn = Quat(axis.x, axis.y, axis.z, cos(angle * timeStep * 0.5f))
        val orn0 = curTrans.getRotation()

        val predictedOrn = dorn * orn0
        predictedOrn.safeNormalize()

        if (predictedOrn.length2() > Float.EPSILON)
            predictedTransform setRotation predictedOrn
        else
            predictedTransform.basis put curTrans.basis
    }

    fun calculateVelocityQuaternion(pos0: Vec3, pos1: Vec3, orn0: Quat, orn1: Quat, timeStep: Float, linVel: Vec3, angVel: Vec3) {
        linVel put (pos1 - pos0) / timeStep
        val axis = Vec3()
        if (orn0 != orn1) {
            val angle = calculateDiffAxisAngleQuaternion(orn0, orn1, axis)
            axis put axis * angle / timeStep
        } else
            angVel put 0f
    }

    /** JVM specific, angle in return   */
    fun calculateDiffAxisAngleQuaternion(orn0: Quat, orn1a: Quat, axis: Vec3): Float {
        val orn1 = orn0 nearest orn1a
        val dorn = orn1 * orn0.inverse()
        val angle = dorn.angle
        axis.put(dorn.x, dorn.y, dorn.z)
        axis[3] = 0f
        //check for axis length
        val len = axis.length2()
        if (len < Float.EPSILON * Float.EPSILON)
            axis.put(1f, 0f, 0f)
        else
            axis /= sqrt(len)
        return angle
    }

    fun calculateVelocity(transform0: Transform, transform1: Transform, timeStep: Float, linVel: Vec3, angVel: Vec3) {
        linVel put (transform1.origin - transform0.origin) / timeStep
        val axis = Vec3()
        val angle = calculateDiffAxisAngle(transform0, transform1, axis)
        angVel put axis * angle / timeStep
    }

    /** JVM specific, angle in return   */
    fun calculateDiffAxisAngle(transform0: Transform, transform1: Transform, axis: Vec3): Float {
        val dmat = transform1.basis * transform0.basis.inverse()
        val dorn = Quat()
        dmat.getRotation(dorn)

        //  floating point inaccuracy can lead to w component > 1..., which breaks
        dorn.normalize()

        val angle = dorn.angle
        axis.put(dorn.x, dorn.y, dorn.z)
        axis[3] = 0f
        //check for axis length
        val len = axis.length2()
        if (len < Float.EPSILON * Float.EPSILON)
            axis.put(1f, 0f, 0f)
        else
            axis /= sqrt(len)
        return angle
    }

}


/** The ConvexSeparatingDistanceUtil can help speed up convex collision detection by conservatively updating a cached
 *  separating distance/vector instead of re-calculating the closest distance   */
class ConvexSeparatingDistanceUtil(var boundingRadiusA: Float, var boundingRadiusB: Float) {
    val ornA = Quat()
    val ornB = Quat()
    val posA = Vec3()
    val posB = Vec3()

    val separatingNormal = Vec3()

    var separatingDistance = 0f

    fun updateSeparatingDistance(transA: Transform, transB: Transform) {
        val toPosA = transA.origin
        val toPosB = transB.origin
        val toOrnA = transA.getRotation()
        val toOrnB = transB.getRotation()

        if (separatingDistance > 0f) {

            val linVelA = Vec3()
            val angVelA = Vec3()
            val linVelB = Vec3()
            val angVelB = Vec3()
            TransformUtil.calculateVelocityQuaternion(posA, toPosA, ornA, toOrnA, 1f, linVelA, angVelA)
            TransformUtil.calculateVelocityQuaternion(posB, toPosB, ornB, toOrnB, 1f, linVelB, angVelB)
            val maxAngularProjectedVelocity = angVelA.length() * boundingRadiusA + angVelB.length() * boundingRadiusB
            val relLinVel = linVelB - linVelA
            var relLinVelocLength = relLinVel dot separatingNormal
            if (relLinVelocLength < 0f) relLinVelocLength = 0f

            val projectedMotion = maxAngularProjectedVelocity + relLinVelocLength
            separatingDistance -= projectedMotion
        }

        posA put toPosA
        posB put toPosB
        ornA put toOrnA
        ornB put toOrnB
    }

    fun initSeparatingDistance(separatingVector: Vec3, separatingDistance: Float, transA: Transform, transB: Transform) {
        val separatingDistance = separatingDistance

        if (separatingDistance > 0f) {
            separatingNormal put separatingVector

            val toPosA = transA.origin
            val toPosB = transB.origin
            val toOrnA = transA.getRotation()
            val toOrnB = transB.getRotation()
            posA put toPosA
            posB put toPosB
            ornA put toOrnA
            ornB put toOrnB
        }
    }
}