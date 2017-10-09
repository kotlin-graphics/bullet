package bullet.linearMath

import bullet.EPSILON
import bullet.f
import kotlin.math.*

open class Vec3 {

    var x = 0f
    var y = 0f
    var z = 0f
    var w = 0f

    constructor()
    constructor(number: Number) : this(number.f)
    constructor(float: Float) : this(float, float, float)
    constructor(floats: FloatArray) : this(floats[0], floats[1], floats[2])
    constructor(v: Vec3) : this(v.x, v.y, v.z)
    constructor(x: Number, y: Number, z: Number) : this(x.f, y.f, z.f)
    constructor(x: Float, y: Float, z: Float) {
        this.x = x
        this.y = y
        this.z = z
    }

    infix fun put(f: Float) = put(f, f, f)
    infix fun put(floats: FloatArray) = put(floats[0], floats[1], floats[2])

    fun put(x: Number, y: Number, z: Number) = put(x.f, y.f, z.f)
    fun put(x: Float, y: Float, z: Float) {
        this.x = x
        this.y = y
        this.z = z
        w = 0f
    }

    infix fun put(v: Vec3) {
        x = v.x
        y = v.y
        z = v.z
        w = v.w
    }

    operator fun set(index: Int, value: Float) = when (index) {
        0 -> x = value
        1 -> y = value
        2 -> z = value
        3 -> w = value
        else -> throw Error()
    }

    operator fun get(index: Int) = when (index) {
        0 -> x
        1 -> y
        2 -> z
        3 -> w
        else -> throw Error()
    }

    operator fun unaryMinus() = Vec3(-x, -y, -z)
    operator fun plus(v: Vec3) = Vec3(x + v.x, y + v.y, z + v.z)
    operator fun plus(f: Float) = Vec3(x + f, y + f, z + f)
    operator fun minus(v: Vec3) = Vec3(x - v.x, y - v.y, z - v.z)
    operator fun times(v: Vec3) = Vec3(x * v.x, y * v.y, z * v.z)
    operator fun times(f: Float) = Vec3(x * f, y * f, z * f)
    operator fun div(v: Vec3) = Vec3(x / v.x, y / v.y, z / v.z)
    operator fun div(v: Float) = Vec3(x / v, y / v, z / v)

    infix operator fun plusAssign(v: Vec3) {
        x += v.x
        y += v.y
        z += v.z
    }

    infix operator fun minusAssign(v: Vec3) {
        x -= v.x
        y -= v.y
        z -= v.z
    }

    infix operator fun timesAssign(v: Vec3) {
        x *= v.x
        y *= v.y
        z *= v.z
    }

    infix operator fun timesAssign(f: Float) {
        x *= f
        y *= f
        z *= f
    }

    infix operator fun divAssign(v: Vec3) {
        x /= v.x
        y /= v.y
        z /= v.z
    }

    infix operator fun divAssign(v: Float) {
        x /= v
        y /= v
        z /= v
    }

    infix fun dot(v: Vec3) = x * v.x + y * v.y + z * v.z

    /** @return the length of the vector squared */
    fun length2() = dot(this)

    /** @return the length of the vector */
    fun length() = sqrt(length2())

    /** @return the norm (length) of the vector */
    fun norm() = length()

    /** @return the norm (length) of the vector */
    fun safeNorm(): Float {
        val d = length2()
        //workaround for some clang/gcc issue of sqrtf(tiny number) = -INF
        if (d > Float.EPSILON)
            return sqrt(d)
        return 0f
    }

    /** @return the distance squared between the ends of this and another vector
     *  This is symantically treating the vector like a point */
    fun distance2(v: Vec3) = (v - this).length2()

    fun distance(v: Vec3) = (v - this).length()

    fun safeNormalize(): Vec3 {
        val l2 = length2()
        //triNormal.normalize();
        if (l2 >= Float.EPSILON * Float.EPSILON)
            div(sqrt(l2))
        else
            put(1f, 0f, 0f)
        return this
    }

    /** Normalize this vector
     * x^2 + y^2 + z^2 = 1 */
    fun normalize(): Vec3 {
        assert(!fuzzyZero())
        val s = 1 / length()
        x *= s
        y *= s
        z *= s
        return this
    }

    fun normalized() = Vec3(this).normalize()

    /**@return a rotated version of this vector
     * @param wAxis The axis to rotate about
     * @param angle The angle to rotate by */
    fun rotate(wAxis: Vec3, angle: Float): Vec3 {
        // wAxis must be a unit lenght vector
        val o = wAxis * (wAxis dot this) // TODO check
        val _x = this - o
        val _y = wAxis cross this

        return o + _x * cos(angle) + _y * sin(angle)
    }

    /** @return the angle between this and another vector
     *  @param v The other vector */
    fun angle(v: Vec3): Float {
        val s = sqrt(length2() * v.length2())
        return acos(dot(v) / s)
    }

    /** @return a vector will the absolute values of each element */
    fun absolute() = Vec3(abs(x), abs(y), abs(z))

    /** @return the cross product between this and another vector
     *  @param v The other vector   */
    infix fun cross(v: Vec3) = Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x)

    fun triple(v1: Vec3, v2: Vec3) = x * (v1.y * v2.z - v1.z * v2.y) + y * (v1.z * v2.x - v1.x * v2.z) + z * (v1.x * v2.y - v1.y * v2.x)

    /** @return the axis with the smallest value
     *  Note return values are 0,1,2 for x, y, or z */
    fun minAxis() = if (x < y) if (x < z) 0 else 2 else if (y < z) 1 else 2

    /** @return the axis with the largest value
     *  Note return values are 0,1,2 for x, y, or z */
    fun maxAxis() = if (x < y) if (y < z) 2 else 1 else if (x < z) 2 else 0

    fun furthestAxis() = absolute().minAxis()
    fun closestAxis() = absolute().maxAxis()

    fun setInterpolate3(v0: Vec3, v1: Vec3, rt: Float) {
        val s = 1f - rt
        x = s * v0.x + rt * v1.x
        y = s * v0.y + rt * v1.y
        z = s * v0.z + rt * v1.z
        //don't do the unused w component
        //		m_co[3] = s * v0[3] + rt * v1[3];
    }

    /** @return the linear interpolation between this and another vector
     *  @param v The other vector
     *  @param t The ration of this to v (t = 0 => return this, t=1 => return other)    */
    fun lerp(v: Vec3, t: Float) = Vec3(x + (v.x - x) * t, y + (v.y - y) * t, z + (v.z - z) * t)

    override fun equals(other: Any?) = other is Vec3 && x == other.x && y == other.y && z == other.z && w == other.w
    override fun hashCode() = 31 * (31 * (31 * x.hashCode() + y.hashCode()) + z.hashCode()) + w.hashCode()

    /** Set each element to the max of the current values and the values of another Vec3
     *  @param other The other Vec3 to compare with */
    fun setMax(other: Vec3) {
        x = max(x, other.x)
        y = max(y, other.y)
        z = max(z, other.z)
        w = max(w, other.w)
    }

    /** Set each element to the min of the current values and the values of another Vec3
     *  @param other The other Vec3 to compare with */
    fun setMin(other: Vec3) {
        x = min(x, other.x)
        y = min(y, other.y)
        z = min(z, other.z)
        w = min(w, other.w)
    }

    fun getSkewSymmetricMatrix(v0: Vec3, v1: Vec3, v2: Vec3) {
        v0.put(0f, -z, y)
        v1.put(z, 0f, -x)
        v2.put(-y, x, 0f)
    }

    val isZero get() = x == 0f && y == 0f && z == 0f

    fun fuzzyZero() = length2() < Float.EPSILON * Float.EPSILON

    /** @returns index of maximum dot product between this and vectors in array[]
     *  @param array The other vectors
     *  @param dotOut The maximum dot product */
    fun maxDot(array: Array<Vec3>, arrayCount: Int, dotOut: FloatArray): Int {

        var maxDot1 = -Float.MAX_VALUE
        var ptIndex = -1
        for (i in 0 until arrayCount) {
            val dot = array[i] dot this
            if (dot > maxDot1) {
                maxDot1 = dot
                ptIndex = i
            }
        }
        dotOut[0] = maxDot1
        return ptIndex
    }

    /** @returns index of minimum dot product between this and vectors in array[]
     *  @param array The other vectors
     *  @param dotOut The minimum dot product */
    fun minDot(array: Array<Vec3>, arrayCount: Int, dotOut: FloatArray): Int {
        var minDot = Float.MAX_VALUE
        var ptIndex = -1
        for (i in 0 until arrayCount) {
            val dot = array[i] dot this
            if (dot < minDot) {
                minDot = dot
                ptIndex = i
            }
        }
        dotOut[0] = minDot
        return ptIndex
    }

    /** create a vector as  Vec3( this->dot( Vec3 v0 ), this->dot( Vec3 v1), this->dot( Vec3 v2 ))  */
    fun dot3(v0: Vec3, v1: Vec3, v2: Vec3) = Vec3(dot(v0), dot(v1), dot(v2))
}

operator fun Float.times(v: Vec3) = Vec3(v.x * this, v.y * this, v.z * this)