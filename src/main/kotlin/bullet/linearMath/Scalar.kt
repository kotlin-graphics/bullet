package bullet.linearMath

import kotlin.math.sqrt


val LARGE_FLOAT = 1e18f


val PI = kotlin.math.PI.toFloat()
val PI2 = PI * 2
val HALF_PI = PI * 0.5f
val SIMDSQRT12 = 0.7071067811865475244008443621048490f
/* reciprocal square root */
fun recipSqrt(x: Float) = 1f / sqrt(x)

fun recip(x: Float) = 1f / x

fun fsels(a: Float, b: Float, c: Float) = if (a >= 0) b else c


/** @returns normalized value in range [-SIMD_PI, SIMD_PI]   */
fun normalizeAngle(angleInRadians: Float): Float {
    val angleInRadians = angleInRadians % PI2
    return when {
        angleInRadians < -PI -> angleInRadians + PI2
        angleInRadians > PI -> angleInRadians - PI2
        else -> angleInRadians
    }
}

/** rudimentary class to provide type info  */
open class TypedObject(val objectType: Int)

fun equal(a: Float, eps: Float) = a <= eps && !(a < -eps)