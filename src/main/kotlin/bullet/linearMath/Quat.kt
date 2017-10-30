package bullet.linearMath

import bullet.EPSILON
import kotlin.math.*

class Quat {

    var x = 0f
    var y = 0f
    var z = 0f
    var w = 0f

    constructor()

    /** Constructor from scalars */
    constructor(x: Float, y: Float, z: Float, w: Float) {
        put(x, y, z, w)
    }

    /** Axis angle Constructor
     * @param axis The axis which the rotation is around
     * @param angle The magnitude of the rotation around the angle (Radians) */
    constructor(axis: Vec3, angle: Float) {
        setRotation(axis, angle)
    }

    /** Constructor from Euler angles
     * @param yaw Angle around Y unless BT_EULER_DEFAULT_ZYX defined then Z
     * @param pitch Angle around X unless BT_EULER_DEFAULT_ZYX defined then Y
     * @param roll Angle around Z unless BT_EULER_DEFAULT_ZYX defined then X */
    constructor(yaw: Float, pitch: Float, roll: Float) {
        setEuler(yaw, pitch, roll)
    }

    /** Set the rotation using axis angle notation
     * @param axis The axis around which to rotate
     * @param angle The magnitude of the rotation in Radians */
    fun setRotation(axis: Vec3, angle: Float) {
        val d = axis.length()
        val s = sin(angle * 0.5f) / d
        put(axis.x * s, axis.y * s, axis.z * s, cos(angle * 0.5f))
    }

    /** Set the quaternion using Euler angles
     * @param yaw Angle around Y
     * @param pitch Angle around X
     * @param roll Angle around Z */
    fun setEuler(yaw: Float, pitch: Float, roll: Float) {
        val halfYaw = yaw * 0.5f
        val halfPitch = pitch * 0.5f
        val halfRoll = roll * 0.5f
        val cosYaw = cos(halfYaw)
        val sinYaw = sin(halfYaw)
        val cosPitch = cos(halfPitch)
        val sinPitch = sin(halfPitch)
        val cosRoll = cos(halfRoll)
        val sinRoll = sin(halfRoll)
        put(
                cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
                cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
                sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
                cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw)
    }

    /** Set the quaternion using euler angles
     * @param yaw Angle around Z
     * @param pitch Angle around Y
     * @param roll Angle around X */
    fun setEulerZYX(yawZ: Float, pitchY: Float, rollX: Float) {
        val halfYaw = yawZ * 0.5f
        val halfPitch = pitchY * 0.5f
        val halfRoll = rollX * 0.5f
        val cosYaw = cos(halfYaw)
        val sinYaw = sin(halfYaw)
        val cosPitch = cos(halfPitch)
        val sinPitch = sin(halfPitch)
        val cosRoll = cos(halfRoll)
        val sinRoll = sin(halfRoll)
        put(
                sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, //x
                cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, //y
                cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw, //z
                cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw) //formerly yzx
    }

    /** Get the euler angles from this quaternion
     * @param yaw Angle around Z
     * @param pitch Angle around Y
     * @param roll Angle around X */
    fun getEulerZYX(yawZ_pitchY_rollX: FloatArray) {
        val sqx = x * x
        val sqy = y * y
        val sqz = z * z
        val squ = w * w
        yawZ_pitchY_rollX[3] = atan2(2 * (y * z + w * x), squ - sqx - sqy + sqz)
        val sarg = -2f * (x * z - w * y)
        yawZ_pitchY_rollX[1] = if (sarg <= -1f) -0.5f * PI else if (sarg >= 1f) 0.5f * PI else asin(sarg)
        yawZ_pitchY_rollX[0] = atan2(2 * (x * y + w * z), squ + sqx - sqy - sqz)
    }

    infix fun put(other: Quat) {
        x = other.x
        y = other.y
        z = other.z
        w = other.w
    }

    fun put(x: Float, y: Float, z: Float, w: Float = 0f) {
        this.x = x
        this.y = y
        this.z = z
        this.w = w
    }

    operator fun plus(q: Quat) = Quat(x + q.x, y + q.y, z + q.z, w + q.w)
    operator fun minus(q: Quat) = Quat(x - q.x, y - q.y, z - q.z, w - q.w)
    operator fun times(f: Float) = Quat(x * f, y * f, z * f, w * f)
    operator fun times(v: Vec3) = Quat(
            w * v.x + y * v.z - z * v.y,
            w * v.y + z * v.x - x * v.z,
            w * v.z + x * v.y - y * v.x,
            -x * v.x - y * v.y - z * v.z)

    operator fun timesAssign(q: Quat) = put(
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y + y * q.w + z * q.x - x * q.z,
            w * q.z + z * q.w + x * q.y - y * q.x,
            w * q.w - x * q.x - y * q.y - z * q.z)

    operator fun times(q: Quat) = Quat(
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y + y * q.w + z * q.x - x * q.z,
            w * q.z + z * q.w + x * q.y - y * q.x,
            w * q.w - x * q.x - y * q.y - z * q.z)

    operator fun div(f: Float) = this * (1f / f)

    operator fun unaryMinus() = Quat(-x, -y, -z, -w)

    /** @return the dot product between this quaternion and another
     *  @param q The other quaternion */
    infix fun dot(q: Quat) = x * q.x + y * q.y + z * q.z + w * q.w

    /** @return the length squared of the quaternion */
    fun length2() = dot(this)

    /** @return the length of the quaternion */
    fun length() = sqrt(length2())

    fun safeNormalize(): Quat {
        val l2 = length2()
        if (l2 > Float.EPSILON) normalize()
        return this
    }

    /** Normalize the quaternion, such that x^2 + y^2 + z^2 +w^2 = 1    */
    fun normalize() {
        val s = 1f / length()
        x *= s
        y *= s
        z *= s
        w *= s
    }

    /** @return a normalized version of this quaternion */
    fun normalized() = this / length()

    /** @return the ***half*** angle between this quaternion and the other
     *  @param q The other quaternion */
    fun angle(q: Quat) = acos(dot(q) / sqrt(length2() * q.length2()))

    /** Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
     *  @return the angle between this quaternion and the other along the shortest path
     *  @param q The other quaternion */
    fun angleShortestPath(q: Quat) = acos(dot(if (dot(q) < 0) -q else q) / sqrt(length2() * q.length2())) * 2f

    /** @return the angle [0, 2Pi] of rotation represented by this quaternion */
    val angle get() = 2f * acos(w)

    /** @return the angle [0, Pi] of rotation represented by this quaternion along the shortest path */
    val angleShortestPath get() = 2f * acos(if (w >= 0) w else -w)

    /** @return the axis of the rotation represented by this quaternion */
    fun getAxis(): Vec3 {
        val sSquared = 1f - w * w
        if (sSquared < 10f * Float.EPSILON) //Check for divide by zero
            return Vec3(1f, 0f, 0f)  // Arbitrary
        val s = 1f / sqrt(sSquared)
        return Vec3(x * s, y * s, z * s)
    }

    /** @return the inverse of this quaternion */
    fun inverse() = Quat(-x, -y, -z, w)

    /** @todo document this and it's use */
    fun farthest(qd: Quat): Quat {
        val diff = this - qd
        val sum = this + qd
        return if (diff dot diff > sum dot sum) qd else -qd
    }

    /**@todo document this and it's use */
    infix fun nearest(qd: Quat): Quat {
        val diff = this - qd
        val sum = this + qd
        return if (diff dot diff < sum dot sum) qd else -qd
    }

    /** @return the quaternion which is the result of Spherical Linear Interpolation between this and the other quaternion
     *  @param q The other quaternion to interpolate with
     *  @param t The ratio between this and q to interpolate.  If t = 0 the result is this, if t=1 the result is q.
     *  Slerp interpolates assuming constant velocity.  */
    fun slerp(q: Quat, t: Float): Quat {

        val magnitude = sqrt(length2() * q.length2())
        assert(magnitude > 0f)

        val product = dot(q) / magnitude
        val absproduct = abs(product)

        if (absproduct < (1f - Float.EPSILON)) {
            // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
            val theta = acos(absproduct)
            val d = sin(theta)
            assert(d > 0f)

            val sign = if (product < 0) -1f else 1f
            val s0 = sin((1f - t) * theta) / d
            val s1 = sin(sign * t * theta) / d

            return Quat(
                    (x * s0 + q.x * s1),
                    (y * s0 + q.y * s1),
                    (z * s0 + q.z * s1),
                    (w * s0 + q.w * s1))
        } else return this
    }

    infix fun rotate(v: Vec3): Vec3 {
        val q = this * v
        q *= this.inverse()
        return Vec3(q.x, q.y, q.z)
    }

    override fun equals(other: Any?) = other is Quat && x == other.x && y == other.y && z == other.z && w == other.w
    override fun hashCode() = 31 * (31 * (31 * x.hashCode() + y.hashCode()) + z.hashCode()) + w.hashCode()
}

/** Game Programming Gems 2.10. make sure v0,v1 are normalized  */
fun shortestArcQuat(v0: Vec3, v1: Vec3): Quat {
    val c = v0 cross v1
    val d = v0 dot v1

    if (d < -1f + Float.EPSILON) {
        val n = Vec3()
        val unused = Vec3()
        planeSpace1(v0, n, unused)
        return Quat(n.x, n.y, n.z, 0f) // just pick any vector that is orthogonal to v0
    }
    val s = sqrt((1f + d) * 2f)
    val rs = 1f / s
    return Quat(c.x * rs, c.y * rs, c.z * rs, s * 0.5f)
}

fun shortestArcQuatNormalize2(v0: Vec3, v1: Vec3): Quat {
    v0.normalize()
    v1.normalize()
    return shortestArcQuat(v0, v1)
}