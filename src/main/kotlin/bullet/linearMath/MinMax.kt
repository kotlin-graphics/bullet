package bullet.linearMath

infix fun Float.min(b: Float) = if (this < b) this else b
infix fun Float.max(b: Float) = if (this > b) this else b
