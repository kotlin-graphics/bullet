package bullet.collision.collisionDispatch

import bullet.L

class HashedSimplePairCache : HashMap<Long, Any?>() {

    val Int.uL get() = L and 0xffffffffL

    operator fun get(indexA: Int, indexB: Int): Any? {
        val a = indexA.uL shl 32
        val b = indexB.uL
        return get(a and b)
    }

    operator fun set(indexA: Int, indexB: Int, value: Any?) {
        val a = indexA.uL shl 32
        val b = indexB.uL
        return set(a and b, value)
    }
}

class SimplePair(val indexA: Int, val indexB: Int) {
    var userPointer: Any? = null
    var userValue
        get() = userPointer as Int
        set(value) {
            userPointer = value
        }
}