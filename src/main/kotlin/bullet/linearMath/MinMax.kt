package bullet.linearMath

infix fun Float.min(b: Float) = if (this < b) this else b
infix fun Float.max(b: Float) = if (this > b) this else b

fun clamped(a: Float, lb: Float, ub: Float) = if (a < lb) lb else if (ub < a) ub else a