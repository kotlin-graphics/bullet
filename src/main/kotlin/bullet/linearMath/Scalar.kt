package bullet.linearMath


val LARGE_FLOAT = 1e18f


val PI = kotlin.math.PI.toFloat()
val PI2 = PI * 2
val HALF_PI = PI * 0.5f

fun fsels(a:Float, b:Float, c:Float) = if(a >= 0) b else c