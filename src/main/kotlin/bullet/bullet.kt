package bullet

import bullet.collision.broadphaseCollision.*
import bullet.dynamics.constraintSolver.SolverConstraint
import bullet.dynamics.constraintSolver.TypedConstraint
import bullet.linearMath.Vec3


//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//
// Constants
//
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//#define DBL_DECIMAL_DIG  17                      // # of decimal digits of rounding precision
//#define DBL_DIG          15                      // # of decimal digits of precision
//#define DBL_EPSILON      2.2204460492503131e-016 // smallest such that 1.0+DBL_EPSILON != 1.0
//#define DBL_HAS_SUBNORM  1                       // type does support subnormal numbers
//#define DBL_MANT_DIG     53                      // # of bits in mantissa
//#define DBL_MAX          1.7976931348623158e+308 // max value
//#define DBL_MAX_10_EXP   308                     // max decimal exponent
//#define DBL_MAX_EXP      1024                    // max binary exponent
//#define DBL_MIN          2.2250738585072014e-308 // min positive value
//#define DBL_MIN_10_EXP   (-307)                  // min decimal exponent
//#define DBL_MIN_EXP      (-1021)                 // min binary exponent
//#define _DBL_RADIX       2                       // exponent radix
//#define DBL_TRUE_MIN     4.9406564584124654e-324 // min positive value
//
//#define FLT_DECIMAL_DIG  9                       // # of decimal digits of rounding precision
//#define FLT_DIG          6                       // # of decimal digits of precision
/** smallest such that 1.0+FLT_EPSILON != 1.0   */
val Float.Companion.EPSILON get() = 1.192092896e-07F
//#define FLT_HAS_SUBNORM  1                       // type does support subnormal numbers
//#define FLT_GUARD        0
//#define FLT_MANT_DIG     24                      // # of bits in mantissa
//#define FLT_MAX          3.402823466e+38F        // max value
//#define FLT_MAX_10_EXP   38                      // max decimal exponent
//#define FLT_MAX_EXP      128                     // max binary exponent
//#define FLT_MIN          1.175494351e-38F        // min normalized positive value
//#define FLT_MIN_10_EXP   (-37)                   // min decimal exponent
//#define FLT_MIN_EXP      (-125)                  // min binary exponent
//#define FLT_NORMALIZE    0
//#define FLT_RADIX        2                       // exponent radix
//#define FLT_TRUE_MIN     1.401298464e-45F        // min positive value
//
//#define LDBL_DIG         DBL_DIG                 // # of decimal digits of precision
//#define LDBL_EPSILON     DBL_EPSILON             // smallest such that 1.0+LDBL_EPSILON != 1.0
//#define LDBL_HAS_SUBNORM DBL_HAS_SUBNORM         // type does support subnormal numbers
//#define LDBL_MANT_DIG    DBL_MANT_DIG            // # of bits in mantissa
//#define LDBL_MAX         DBL_MAX                 // max value
//#define LDBL_MAX_10_EXP  DBL_MAX_10_EXP          // max decimal exponent
//#define LDBL_MAX_EXP     DBL_MAX_EXP             // max binary exponent
//#define LDBL_MIN         DBL_MIN                 // min normalized positive value
//#define LDBL_MIN_10_EXP  DBL_MIN_10_EXP          // min decimal exponent
//#define LDBL_MIN_EXP     DBL_MIN_EXP             // min binary exponent
//#define _LDBL_RADIX      _DBL_RADIX              // exponent radix
//#define LDBL_TRUE_MIN    DBL_TRUE_MIN            // min positive value
//
//#define DECIMAL_DIG      DBL_DECIMAL_DIG

val Boolean.i get() = if (this) 1 else 0
val Byte.i get() = toInt()
val Int.f get() = toFloat()
val Int.L get() = toLong()
val Int.bool get() = this != 0
val Int.s get() = toShort()
val Short.i get() = toInt()
val Number.f get() = toFloat()
val Long.i get() = toInt()
val Float.s get() = toShort()
val Float.i get() = toInt()
val Char.i get() = toInt()

infix fun Int.has(b: Int) = (this and b) != 0
infix fun Int.hasnt(b: Int) = (this and b) == 0
infix fun Int.wo(b: Int) = this and b.inv()

val Int.Companion.BYTES get() = 4
val Float.Companion.BYTES get() = 4

val DEBUG = true
val DEBUG_DRAW = false


infix fun <T> ArrayList<T>.push(element: T) = add(element)
fun <T> ArrayList<T>.pop(): T {
    val last = last()
    remove(last)
    return last
}

fun <T> ArrayList<T>.swapLastAt(index: Int) = swap(index, lastIndex)

fun <T> ArrayList<T>.swap(index0: Int, index1: Int) {
    val e = get(index0)
    set(index0, get(index1))
    set(index1, e)
}

infix fun <T> ArrayList<T>.resize(newSize: Int) {
    when {
        size > newSize -> for (i in newSize until size) pop()
        newSize > size -> when (get(0)) {
            is Dbvt.StkNN? -> for (i in size until newSize) add(null as T)
            is Dbvt.StkNN -> for (i in size until newSize) add(Dbvt.StkNN() as T)
            is DbvtNode -> for (i in size until newSize) add(DbvtNode() as T)
            is BroadphasePair -> for (i in size until newSize) add(BroadphasePair() as T)
            is TypedConstraint.ConstraintInfo1 -> for (i in size until newSize) add(TypedConstraint.ConstraintInfo1() as T)
            is SolverConstraint -> for (i in size until newSize) add(SolverConstraint() as T)
            is Int -> for (i in size until newSize) add(0 as T)
            is Vec3 -> for (i in size until newSize) add(Vec3() as T)
            is QuantizedBvhNode -> for (i in size until newSize) add(QuantizedBvhNode() as T)
            is CollisionAlgorithm? -> for (i in size until newSize) add(null as T)
        }
    }
}


val DISABLE_DBVT_COMPOUNDSHAPE_RAYCAST_ACCELERATION = false
val COMPARE_BTRAY_AABB2 = false
val USE_PATH_COMPRESSION = true
val USE_STATIC_ONLY = false
val BT_NO_PROFILE = true
val USE_SEPDISTANCE_UTIL2 = false
val DISABLE_CAPSULE_CAPSULE_COLLIDER = true
val TEST_INTERNAL_OBJECTS = true
val ONLY_REPORT_DEEPEST_POINT = false
val ZERO_MARGIN = false
val DEBUG_CONTACTS = false
val CLEAR_MANIFOLD = true
val USE_PERSISTENT_CONTACTS = true
val USE_CENTER_POINT = false


/** internal debugging variable. this value shouldn't be too high */
var gNumClampedCcdMotions = 0


fun BT_PROFILE(text: String){
    if(DEBUG) println(text)
}