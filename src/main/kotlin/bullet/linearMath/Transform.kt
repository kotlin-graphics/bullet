package bullet.linearMath

/** The Transform class supports rigid transforms with only translation and rotation and no scaling/shear.
 *  It can be used in combination with btVector3, btQuaternion and btMatrix3x3 linear algebra classes   */
class Transform {

    /** Storage for the rotation    */
    var basis = Mat3()
    /** Storage for the translation */
    var origin = Vec3()

    constructor()

    /** @constructor from btQuaternion (optional btVector3 )
     *  @param q Rotation from quaternion
     *  @param c Translation from Vector (default 0,0,0) */
    constructor(q: Quat, c: Vec3 = Vec3()) {
        basis = Mat3(q)
        origin put c
    }

    /** @constructor from btMatrix3x3 (optional btVector3)
     *  @param b Rotation from Matrix
     *  @param c Translation from Vector default (0,0,0)*/
    constructor(b: Mat3, c: Vec3 = Vec3()) {
        basis put b
        origin put c
    }

    /** @constructor Copy constructor */
    constructor(other: Transform) {
        basis put other.basis
        origin put other.origin
    }

    /** Set the current transform as the value of the product of two transforms
     *  @param t1 Transform 1
     *  @param t2 Transform 2
     *  This = Transform1 * Transform2 */
    fun mult(t1: Transform, t2: Transform) {
        basis = t1.basis * t2.basis
        origin = t1 * t2.origin
    }

    /*		void multInverseLeft(const btTransform& t1, const btTransform& t2) {
			btVector3 v = t2.m_origin - t1.m_origin;
			m_basis = btMultTransposeLeft(t1.m_basis, t2.m_basis);
			m_origin = v * t1.m_basis;
		}
		*/

    /** @return the transform of the vector */
    operator fun times(x: Vec3) = x.dot3(basis[0], basis[1], basis[2]) + origin

    operator fun invoke(x: Vec3) = x.dot3(basis[0], basis[1], basis[2]) + origin

    /** @return the transform of the btQuaternion */
    operator fun times(q: Quat) = getRotation() * q

    /** @return a quaternion representing the rotation */
    fun getRotation(): Quat {
        val q = Quat()
        basis.getRotation(q)
        return q
    }

    /** Set from an array
     *  @param m A pointer to a 16 element array (12 rotation(row major padded on the right by 1), and 3 translation */
    fun setFromOpenGLMatrix(m: FloatArray) {
        basis.setFromOpenGLSubMatrix(m)
        origin.put(m[12], m[13], m[14])
    }

    /** Fill an array representation
     *  @param m A pointer to a 16 element array (12 rotation(row major padded on the right by 1), and 3 translation */
    fun getOpenGLMatrix(m: FloatArray) {
        basis.getOpenGLSubMatrix(m)
        m[12] = origin.x
        m[13] = origin.y
        m[14] = origin.z
        m[15] = 1f
    }

    infix fun invXform(inVec: Vec3) = basis.transpose() * (inVec - origin)

    /** Set the rotational element by Quat  */
    infix fun setRotation(q: Quat) = basis.setRotation(q)

    /** Set this transformation to the identity */
    fun setIdentity() {
        basis.setIdentity()
        origin put 0f
    }

    /** Multiply this Transform by another(this = this * another)
     *  @param t The other transform */
    operator fun times(t: Transform) = Transform(basis * t.basis, this(t.origin))

    /** Return the inverse of this transform */
    fun inverse() = basis.transpose().let { Transform(it, it * -origin) }

    /** Return the inverse of this transform times the other transform
     *  @param t The other transform
     *  return this.inverse() * the other */
    infix fun inverseTimes(t: Transform): Transform {
        val v = t.origin - origin
        return Transform(basis transposeTimes t.basis, v * basis)
    }

    infix fun put(other: Transform) {
        basis put other.basis
        origin put other.origin
    }

    override fun equals(other: Any?) = other is Transform && basis == other.basis && origin == other.origin
    override fun hashCode() = 31 * basis.hashCode() + origin.hashCode()

    companion object {
        /**@brief Return an identity transform */
        val identity get() = Transform(Mat3.identity)
    }
}