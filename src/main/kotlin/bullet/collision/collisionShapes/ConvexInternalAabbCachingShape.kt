package bullet.collision.collisionShapes

import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import bullet.linearMath.transformAabb

/** ConvexInternalAabbCachingShape adds local aabb caching for convex shapes, to avoid expensive bounding box calculations    */
abstract class ConvexInternalAabbCachingShape : ConvexInternalShape() {

    var localAabbMin = Vec3(1)
    var localAabbMax = Vec3(-1)
    var isLocalAabbValid = false

    fun setCachedLocalAabb(aabbMin: Vec3, aabbMax: Vec3) {
        isLocalAabbValid = true
        localAabbMin put aabbMin
        localAabbMax put aabbMax
    }

    fun getCachedLocalAabb(aabbMin: Vec3, aabbMax: Vec3) {
        assert(isLocalAabbValid)
        aabbMin put localAabbMin
        aabbMax put localAabbMax
    }

    fun getNonvirtualAabb(trans: Transform, aabbMin: Vec3, aabbMax: Vec3, margin: Float) {

        //lazy evaluation of local aabb
        assert(isLocalAabbValid)
        transformAabb(localAabbMin, localAabbMax, margin, trans, aabbMin, aabbMax)
    }

    override var localScaling
        get() = super.localScaling
        set(value) {
            super.localScaling = value
            recalcLocalAabb()
        }

    override fun getAabb(trans: Transform, aabbMin: Vec3, aabbMax: Vec3) = getNonvirtualAabb(trans, aabbMin, aabbMax, margin)

    fun recalcLocalAabb() {

        isLocalAabbValid = true

        val _supporting = arrayOf(Vec3(), Vec3(), Vec3(), Vec3(), Vec3(), Vec3())

        batchedUnitVectorGetSupportingVertexWithoutMargin(_directions, _supporting, 6)

        for (i in 0 .. 2)        {
            localAabbMax[i] = _supporting[i][i] + collisionMargin
            localAabbMin[i] = _supporting[i + 3][i] - collisionMargin
        }
    }

    companion object {

        private val _directions = arrayOf(
                Vec3(1f, 0f, 0f),
                Vec3(0f, 1f, 0f),
                Vec3(0f, 0f, 1f),
                Vec3(-1f, 0f, 0f),
                Vec3(0f, -1f, 0f),
                Vec3(0f, 0f, -1f))
    }
}