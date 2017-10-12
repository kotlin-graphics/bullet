package bullet.linearMath

import kotlin.math.abs

// TODO move same place an Bullet
class Vec4 : Vec3 {

    constructor() : super()

    constructor(x: Float, y: Float, z: Float, w: Float) : super(x, y, z) {
        this.w = w
    }

    fun absolute4() = Vec4(abs(x), abs(y), abs(z), abs(w))

    fun maxAxis4(): Int {
        var maxIndex = -1
        var maxVal = -LARGE_FLOAT
        if (x > maxVal) {
            maxIndex = 0
            maxVal = x
        }
        if (y > maxVal) {
            maxIndex = 1
            maxVal = y
        }
        if (z > maxVal) {
            maxIndex = 2
            maxVal = z
        }
        if (w > maxVal) maxIndex = 3
        return maxIndex
    }

    fun minAxis4(): Int {
        var minIndex = -1
        var minVal = LARGE_FLOAT
        if (x < minVal) {
            minIndex = 0
            minVal = x
        }
        if (y < minVal) {
            minIndex = 1
            minVal = y
        }
        if (z < minVal) {
            minIndex = 2
            minVal = z
        }
        if (w < minVal) minIndex = 3
        return minIndex
    }

    fun closestAxis4() = absolute4().maxAxis4()

    fun put(x: Float, y: Float, z: Float, w: Float) {
        this.x = x
        this.y = y
        this.z = z
        this.w = w
    }
}