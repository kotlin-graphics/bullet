package bullet.linearMath

/** The Transform class supports rigid transforms with only translation and rotation and no scaling/shear.
 *  It can be used in combination with btVector3, btQuaternion and btMatrix3x3 linear algebra classes   */
class Transform {

    /** Storage for the rotation    */
    var basis = Mat3()
    /** Storage for the translation */
    var origin = Vec3()
}