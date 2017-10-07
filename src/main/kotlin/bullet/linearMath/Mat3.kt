package bullet.linearMath

/** The Mat3 class implements a 3x3 rotation matrix, to perform linear algebra in combination with Quat, Transform and
 *  Vec3.
 *  Make sure to only include a pure orthogonal matrix without scaling. */
class Mat3 {

    /** Data storage for the matrix, each vector is a row of the matrix */
    var el = Array(3, { Vec3() })

    /** Constructor from Quaternion */
    constructor(q:Quat)

//    constructor(const btScalar& yaw, const btScalar& pitch, const btScalar& roll) ) setEulerYPR(yaw, pitch, roll)

    constructor(xx:Float, xy: Float, xz: Float, yx:Float, yy:Float, yz:Float, zx:Float, zy:Float, zz:Float) {
        put(xx, xy, xz, yx, yy, yz, zx, zy, zz)
    }

    /** Copy constructor */
    constructor(other:Mat3)    {
        el[0] put other.el[0]
        el[1] put other.el[1]
        el[2] put other.el[2]
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
    fun put(xx:Float, xy: Float, xz: Float, yx:Float, yy:Float, yz:Float, zx:Float, zy:Float, zz:Float) {
        el[0].put(xx, xy, xz)
        el[1].put(yx, yy, yz)
        el[2].put(zx, zy, zz)
    }
}