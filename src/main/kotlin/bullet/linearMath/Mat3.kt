package bullet.linearMath

import bullet.EPSILON
import kotlin.math.*

/** The Mat3 class implements a 3x3 rotation matrix, to perform linear algebra in combination with Quat, Transform and
 *  Vec3.
 *  Make sure to only include a pure orthogonal matrix without scaling. */
class Mat3 {

    /** Data storage for the matrix, each vector is a row of the matrix */
    var el = Array(3, { Vec3() })

    constructor()

    /** Constructor from Quaternion */
    constructor(q: Quat)

//    constructor(const btScalar& yaw, const btScalar& pitch, const btScalar& roll) ) setEulerYPR(yaw, pitch, roll)

    constructor(xx: Float, xy: Float, xz: Float, yx: Float, yy: Float, yz: Float, zx: Float, zy: Float, zz: Float) {
        put(xx, xy, xz, yx, yy, yz, zx, zy, zz)
    }

    /** Copy constructor */
    constructor(other: Mat3) {
        el[0] put other.el[0]
        el[1] put other.el[1]
        el[2] put other.el[2]
    }

    /** Get a column of the matrix as a vector
     *  @param i Column number 0 indexed */
    fun getColumn(i: Int) = Vec3(el[0][i], el[1][i], el[2][i])

    /** Get a mutable reference to a row of the matrix as a vector
     *  @param i Row number 0 indexed */
    operator fun get(index: Int) = el[index]

    operator fun get(c: Int, r: Int) = el[c][r]

    /** Set from the rotational part of a 4x4 OpenGL matrix
     *  @param m A pointer to the beginning of the array of scalars*/
    fun setFromOpenGLSubMatrix(m: FloatArray) {
        el[0].put(m[0], m[4], m[8])
        el[1].put(m[1], m[5], m[9])
        el[2].put(m[2], m[6], m[10])
    }

    /** Set the values of the matrix explicitly (row major)
     *  @param xx Top left
     *  @param xy Top Middle
     *  @param xz Top Right
     *  @param yx Middle Left
     *  @param yy Middle Middle
     *  @param yz Middle Right
     *  @param zx Bottom Left
     *  @param zy Bottom Middle
     *  @param zz Bottom Right  */
    fun put(xx: Float, xy: Float, xz: Float, yx: Float, yy: Float, yz: Float, zx: Float, zy: Float, zz: Float) {
        el[0].put(xx, xy, xz)
        el[1].put(yx, yy, yz)
        el[2].put(zx, zy, zz)
    }

    infix fun put(b: Mat3) = put(
            b[0, 0], b[0, 1], b[0, 2],
            b[1, 0], b[1, 1], b[1, 2],
            b[2, 0], b[2, 1], b[2, 2])

    /** Set the matrix from a quaternion
     *  @param q The Quaternion to match */
    fun setRotation(q: Quat) {
        val d = q.length2()
        val s = 2f / d

        val xs = q.x * s
        val ys = q.y * s
        val zs = q.z * s
        val wx = q.w * xs
        val wy = q.w * ys
        val wz = q.w * zs
        val xx = q.x * xs
        val xy = q.x * ys
        val xz = q.x * zs
        val yy = q.y * ys
        val yz = q.y * zs
        val zz = q.z * zs
        put(
                1f - (yy + zz), xy - wz, xz + wy,
                xy + wz, 1f - (xx + zz), yz - wx,
                xz - wy, yz + wx, 1f - (xx + yy))
    }

    /** Set the matrix from euler angles using YPR around YXZ respectively
     *  @param yaw Yaw about Y axis
     *  @param pitch Pitch about X axis
     *  @param roll Roll about Z axis     */
    fun setEulerYPR(yaw: Float, pitch: Float, roll: Float) = setEulerZYX(roll, pitch, yaw)

    /** Set the matrix from euler angles YPR around ZYX axes
     * @param eulerX Roll about X axis
     * @param eulerY Pitch around Y axis
     * @param eulerZ Yaw aboud Z axis
     *
     * These angles are used to produce a rotation matrix. The euler angles are applied in ZYX order.
     * I.e a vector is first rotated about X then Y and then Z  */
    fun setEulerZYX(eulerX: Float, eulerY: Float, eulerZ: Float) {
        //@todo proposed to reverse this since it's labeled zyx but takes arguments xyz and it will match all other parts of the code
        val ci = cos(eulerX)
        val cj = cos(eulerY)
        val ch = cos(eulerZ)
        val si = sin(eulerX)
        val sj = sin(eulerY)
        val sh = sin(eulerZ)
        val cc = ci * ch
        val cs = ci * sh
        val sc = si * ch
        val ss = si * sh
        put(cj * ch, sj * sc - cs, sj * cc + ss,
                cj * sh, sj * ss + cc, sj * cs - sc,
                -sj, cj * si, cj * ci)
    }

    /** Set the matrix to the identity */
    fun setIdentity() = put(1f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 1f)

    /** Fill the rotational part of an OpenGL matrix and clear the shear/perspective
     * @param m The array to be filled */
    fun getOpenGLSubMatrix(m: FloatArray) {
        m[0] = el[0].x
        m[1] = el[1].x
        m[2] = el[2].x
        m[3] = 0f
        m[4] = el[0].y
        m[5] = el[1].y
        m[6] = el[2].y
        m[7] = 0f
        m[8] = el[0].z
        m[9] = el[1].z
        m[10] = el[2].z
        m[11] = 0f
    }

    /** Get the matrix represented as a quaternion
     * @param q The quaternion which will be set */
    fun getRotation(q: Quat) {

        val trace = el[0].x + el[1].y + el[2].z

        val temp = FloatArray(4)

        if (trace > 0f) {
            var s = sqrt(trace + 1f)
            temp[3] = (s * 0.5f)
            s = 0.5f / s

            temp[0] = (el[2].y - el[1].z) * s
            temp[1] = (el[0].z - el[2].x) * s
            temp[2] = (el[1].x - el[0].y) * s
        } else {
            val i = when {
                el[0].x < el[1].y -> if (el[1].y < el[2].z) 2 else 1
                else -> if (el[0].x < el[2].z) 2 else 0
            }
            val j = (i + 1) % 3
            val k = (i + 2) % 3

            var s = sqrt(el[i][i] - el[j][j] - el[k][k] + 1f)
            temp[i] = s * 0.5f
            s = 0.5f / s

            temp[3] = (el[k][j] - el[j][k]) * s
            temp[j] = (el[j][i] + el[i][j]) * s
            temp[k] = (el[k][i] + el[i][k]) * s
        }
        q.put(temp[0], temp[1], temp[2], temp[3])
    }

    /** Get the matrix represented as euler angles around YXZ, roundtrip with setEulerYPR
     * @param yaw Yaw around Y axis
     * @param pitch Pitch around X axis
     * @param roll around Z axis */
    fun getEulerYPR(yawPitchRoll: FloatArray) {

        // first use the normal calculus
        var yaw = atan2(el[1].x, el[0].x)
        val pitch = asin(-el[2].x)
        var roll = atan2(el[2].y, el[2].z)

        // on pitch = +/-HalfPI
        if (abs(pitch) == HALF_PI) {
            if (yaw > 0) yaw -= PI
            else yaw += PI
            if (roll > 0) roll -= PI
            else roll += PI
        }
        yawPitchRoll[0] = yaw
        yawPitchRoll[1] = pitch
        yawPitchRoll[2] = roll
    }

    /** Get the matrix represented as euler angles around ZYX
     * @param yaw Yaw around X axis
     * @param pitch Pitch around Y axis
     * @param roll around X axis
     * @param solutionNumber Which solution of two possible solutions ( 1 or 2) are possible values*/
    fun getEulerZYX(yawPitchRoll: FloatArray, solutionNumber: Int = 1) {

        val yaw = 0
        val pitch = 1
        val roll = 2

        val eulerOut = FloatArray(3)
        val eulerOut2 = FloatArray(3) //second solution
        //get the pointer to the raw data

        // Check that pitch is not at a singularity
        if (abs(el[2].x) >= 1) {
            eulerOut[yaw] = 0f
            eulerOut2[yaw] = 0f

            // From difference of angles formula
            val delta = atan2(el[0].x, el[0].z)
            if (el[2].x > 0) {  //gimbal locked up
                eulerOut[pitch] = PI / 2f
                eulerOut2[pitch] = PI / 2f
                eulerOut[roll] = eulerOut[pitch] + delta
                eulerOut2[roll] = eulerOut[pitch] + delta
            } else { // gimbal locked down
                eulerOut[pitch] = -PI / 2f
                eulerOut2[pitch] = -PI / 2f
                eulerOut[roll] = -eulerOut[pitch] + delta
                eulerOut2[roll] = -eulerOut[pitch] + delta
            }
        } else {
            eulerOut[pitch] = -asin(el[2].x)
            eulerOut2[pitch] = PI - eulerOut[pitch]

            eulerOut[roll] = atan2(el[2].y / cos(eulerOut[pitch]), el[2].z / cos(eulerOut[pitch]))
            eulerOut2[roll] = atan2(el[2].y / cos(eulerOut2[pitch]), el[2].z / cos(eulerOut2[pitch]))

            eulerOut[yaw] = atan2(el[1].x / cos(eulerOut[pitch]), el[0].x / cos(eulerOut[pitch]))
            eulerOut2[yaw] = atan2(el[1].x / cos(eulerOut2[pitch]), el[0].x / cos(eulerOut2[pitch]))
        }

        if (solutionNumber == 1) {
            yawPitchRoll[yaw] = eulerOut[yaw]
            yawPitchRoll[pitch] = eulerOut[pitch]
            yawPitchRoll[roll] = eulerOut[roll]
        } else {
            yawPitchRoll[yaw] = eulerOut2[yaw]
            yawPitchRoll[pitch] = eulerOut2[pitch]
            yawPitchRoll[roll] = eulerOut2[roll]
        }
    }

    /** Create a scaled copy of the matrix
     * @param s Scaling vector The elements of the vector will scale each column */
    fun scaled(s: Vec3) = Mat3(
            el[0].x * s.x, el[0].y * s.y, el[0].z * s.z,
            el[1].x * s.x, el[1].y * s.y, el[1].z * s.z,
            el[2].x * s.x, el[2].y * s.y, el[2].z * s.z)

    /** @return the determinant of the matrix */
    fun determinant() = el[0].triple(el[1], el[2])

    /** @return the adjoint of the matrix */
    fun adjoint() = Mat3(
            cofac(1, 1, 2, 2), cofac(0, 2, 2, 1), cofac(0, 1, 1, 2),
            cofac(1, 2, 2, 0), cofac(0, 0, 2, 2), cofac(0, 2, 1, 0),
            cofac(1, 0, 2, 1), cofac(0, 1, 2, 0), cofac(0, 0, 1, 1))

    /** @return the matrix with all values non negative */
    fun absolute() = Mat3(
            abs(el[0].x), abs(el[0].y), abs(el[0].z),
            abs(el[1].x), abs(el[1].y), abs(el[1].z),
            abs(el[2].x), abs(el[2].y), abs(el[2].z))

    /** @return the transpose of the matrix */
    fun transpose() = Mat3(
            el[0].x, el[1].x, el[2].x,
            el[0].y, el[1].y, el[2].y,
            el[0].z, el[1].z, el[2].z)

    /** @return the inverse of the matrix */
    fun inverse(): Mat3 {
        val co = Vec3(cofac(1, 1, 2, 2), cofac(1, 2, 2, 0), cofac(1, 0, 2, 1))
        val det = el[0] dot co
        //btFullAssert(det != btScalar(0.0));
        val s = 1f / det
        return Mat3(
                co.x * s, cofac(0, 2, 2, 1) * s, cofac(0, 1, 1, 2) * s,
                co.y * s, cofac(0, 0, 2, 2) * s, cofac(0, 2, 1, 0) * s,
                co.z * s, cofac(0, 1, 2, 0) * s, cofac(0, 0, 1, 1) * s)
    }

    /** Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse in one-shot cases.
     *  Solve33 is from Box2d, thanks to Erin Catto,    */
    fun solve33(b: Vec3): Vec3 {
        val col1 = getColumn(0)
        val col2 = getColumn(1)
        val col3 = getColumn(2)

        var det = col1 dot (col2 cross col3)
        if (abs(det) > Float.EPSILON)
            det = 1f / det
        return Vec3(
                det * b.dot(col2 cross col3),
                det * col1.dot(b cross col3),
                det * col1.dot(col2 cross b))
    }

    infix fun transposeTimes(m: Mat3) = Mat3(
            el[0].x * m[0].x + el[1].x * m[1].x + el[2].x * m[2].x,
            el[0].x * m[0].y + el[1].x * m[1].y + el[2].x * m[2].y,
            el[0].x * m[0].z + el[1].x * m[1].z + el[2].x * m[2].z,
            el[0].y * m[0].x + el[1].y * m[1].x + el[2].y * m[2].x,
            el[0].y * m[0].y + el[1].y * m[1].y + el[2].y * m[2].y,
            el[0].y * m[0].z + el[1].y * m[1].z + el[2].y * m[2].z,
            el[0].z * m[0].x + el[1].z * m[1].x + el[2].z * m[2].x,
            el[0].z * m[0].y + el[1].z * m[1].y + el[2].z * m[2].y,
            el[0].z * m[0].z + el[1].z * m[1].z + el[2].z * m[2].z)

    infix fun timesTranspose(m: Mat3) = Mat3(
            el[0] dot m[0], el[0] dot m[1], el[0] dot m[2],
            el[1] dot m[0], el[1] dot m[1], el[1] dot m[2],
            el[2] dot m[0], el[2] dot m[1], el[2] dot m[2])

    infix fun tdotx(v: Vec3) = el[0].x * v.x + el[1].x * v.y + el[2].x * v.z
    infix fun tdoty(v: Vec3) = el[0].y * v.x + el[1].y * v.y + el[2].y * v.z
    infix fun tdotz(v: Vec3) = el[0].z * v.x + el[1].z * v.y + el[2].z * v.z

    /** extractRotation is from "A robust method to extract the rotational part of deformations"
     *  See http://dl.acm.org/citation.cfm?doid=2994258.2994269 */
    fun extractRotation(q: Quat, tolerance: Float = 1.0e-9f, maxIter: Int = 100) {
        val a = this
        for (iter in 0 until maxIter) {
            val r = Mat3(q)
            val b = r.getColumn(0).cross(a.getColumn(0)) + r.getColumn(1).cross(a.getColumn(1)) +
                    r.getColumn(2).cross(a.getColumn(2))
            val c = r.getColumn(0).dot(a.getColumn(0)) + r.getColumn(1).dot(a.getColumn(1)) +
                    r.getColumn(2).dot(a.getColumn(2))
            val omega = b * (1f / abs(c) + tolerance)
            val w = omega.norm()
            if (w < tolerance) break
            val v = Vec3((1f / w) * omega)
            q.put(v.x * q.x, v.y * q.y, v.z * q.z, w * q.w)
            q.normalize()
        }
    }

    /** Diagonalizes this matrix
     *  @param rot stores the rotation from the coordinate system in which the matrix is diagonal to the original
     *  coordinate system, i.e., old_this = rot * new_this * rot^T.
     *  @param threshold See iteration
     *  @param maxIter The iteration stops when we hit the given tolerance or when maxIter have been executed.     */
    fun diagonalize(rot: Mat3, tolerance: Float = 1.0e-9f, maxIter: Int = 100) {
        val r = Quat(0f, 0f, 0f, 1f)
        extractRotation(r, tolerance, maxIter)
        rot.setRotation(r)
        val rotInv = Mat3(r.inverse())
        put(
                tdotx(rotInv[0]), tdoty(rotInv[0]), tdotz(rotInv[0]),
                tdotx(rotInv[1]), tdoty(rotInv[1]), tdotz(rotInv[1]),
                tdotx(rotInv[2]), tdoty(rotInv[2]), tdotz(rotInv[2]))
    }

    /** Calculate the matrix cofactor
     *  @param r1 The first row to use for calculating the cofactor
     *  @param c1 The first column to use for calculating the cofactor
     *  @param r1 The second row to use for calculating the cofactor
     *  @param c1 The second column to use for calculating the cofactor
     *  See http://en.wikipedia.org/wiki/Cofactor_(linear_algebra) for more details     */
    fun cofac(r1: Int, c1: Int, r2: Int, c2: Int) = el[r1][c1] * el[r2][c2] - el[r1][c2] * el[r2][c1]

    operator fun times(m: Mat3) = Mat3(
            m tdotx el[0], m tdoty el[0], m tdotz el[0],
            m tdotx el[1], m tdoty el[1], m tdotz el[1],
            m tdotx el[2], m tdoty el[2], m tdotz el[2])

    operator fun times(v: Vec3) = Vec3(el[0] dot v, el[1] dot v, el[2] dot v)

    operator fun times(f: Float) = Mat3(
            el[0].x * f, el[0].y * f, el[0].z * f,
            el[1].x * f, el[1].y * f, el[1].z * f,
            el[2].x * f, el[2].y * f, el[2].z * f)

    operator fun plus(m: Mat3) = Mat3(
            el[0][0] + m.el[0][0],
            el[0][1] + m.el[0][1],
            el[0][2] + m.el[0][2],
            el[1][0] + m.el[1][0],
            el[1][1] + m.el[1][1],
            el[1][2] + m.el[1][2],
            el[2][0] + m.el[2][0],
            el[2][1] + m.el[2][1],
            el[2][2] + m.el[2][2])

    operator fun minus(m: Mat3) = Mat3(
            el[0][0] - m.el[0][0],
            el[0][1] - m.el[0][1],
            el[0][2] - m.el[0][2],
            el[1][0] - m.el[1][0],
            el[1][1] - m.el[1][1],
            el[1][2] - m.el[1][2],
            el[2][0] - m.el[2][0],
            el[2][1] - m.el[2][1],
            el[2][2] - m.el[2][2])

    override fun equals(other: Any?) = other is Mat3 && el[0] == other.el[0] && el[1] == other.el[1] && el[2] == other.el[2]
    override fun hashCode() = 31 * (31 * el[0].hashCode() + el[1].hashCode()) + el[2].hashCode()
}

operator fun Vec3.times(m: Mat3) = Vec3(m tdotx this, m tdoty this, m tdotz this)