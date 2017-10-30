package bullet.collision.collisionShapes

import bullet.EPSILON
import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import bullet.linearMath.times
import bullet.collision.broadphaseCollision.BroadphaseNativeTypes as BNT

class SphereShape(radius: Float) : ConvexInternalShape() {

    init {
        shapeType = BNT.SPHERE_SHAPE_PROXYTYPE
        localScaling put 1f
        implicitShapeDimensions.put(0f)
        implicitShapeDimensions.x = radius
        collisionMargin = radius
        padding = 0f
    }

    override fun localGetSupportingVertex(vec: Vec3): Vec3 {
        val supVertex = localGetSupportingVertexWithoutMargin(vec)

        val vecnorm = Vec3(vec)
        if (vecnorm.length2() < Float.EPSILON * Float.EPSILON)
            vecnorm put -1f
        vecnorm.normalize()
        supVertex += margin * vecnorm
        return supVertex
    }

    override fun localGetSupportingVertexWithoutMargin(vec: Vec3) = Vec3()
    /** notice that the vectors should be unit length   */
    override fun batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: Array<Vec3>, supportVerticesOut: Array<Vec3>, numVectors: Int) {
        for (i in 0 until numVectors) supportVerticesOut[i] put 0f
    }


    override fun calculateLocalInertia(mass: Float, inertia: Vec3) {
        inertia.put(0.4f * mass * margin * margin)
    }

    /** broken due to scaling   */
    override fun getAabb(trans: Transform, aabbMin: Vec3, aabbMax: Vec3) {
        val center = trans.origin
        val extent = Vec3(margin)
        aabbMin put (center - extent)
        aabbMax put (center + extent)
    }

    var radius
        get() = implicitShapeDimensions.x * localScaling.x
        set(value) {
            implicitShapeDimensions.x = value
            super.margin = value
        }

    /** debugging   */
    override val name get() = "SPHERE"

    override var margin
        set(value) {
            super.margin = margin
        }
        /* to improve gjk behaviour, use radius + margin as the full margin, so never get into the penetration case
            this means, non-uniform scaling is not supported anymore         */
        get() = radius

}