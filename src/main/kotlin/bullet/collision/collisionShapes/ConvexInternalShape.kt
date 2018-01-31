package bullet.collision.collisionShapes

import bullet.EPSILON
import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import bullet.linearMath.times

abstract class ConvexInternalShape : ConvexShape() {

    /** local scaling. collisionMargin is not scaled !  */
    override var localScaling = Vec3(1)
        set(value) {
            localScaling put value.absolute()
        }

    /** Warning: use setImplicitShapeDimensions with care
     *  changing a collision shape while the body is in the world is not recommended, it is best to remove the body
     *  from the world, then make the change, and re-add it
     *  alternatively flush the contact points, see documentation for 'cleanProxyFromPairs' */
    val implicitShapeDimensions = Vec3()

    var collisionMargin = CONVEX_DISTANCE_MARGIN

    var padding = 0f

    override fun localGetSupportingVertex(vec: Vec3): Vec3 {

        val supVertex = localGetSupportingVertexWithoutMargin(vec)

        if (margin != 0f) {
            val vecnorm = Vec3(vec)
            if (vecnorm.length2() < Float.EPSILON * Float.EPSILON)
                vecnorm put -1f
            vecnorm.normalize()
            supVertex += margin * vecnorm
        }
        return supVertex
    }

    fun setSafeMargin(minDimension: Float, defaultMarginMultiplier: Float = 0.1f) {
        val safeMargin = defaultMarginMultiplier * minDimension
        if (safeMargin < margin) margin = safeMargin
    }

    fun setSafeMargin(halfExtents: Vec3, defaultMarginMultiplier: Float = 0.1f) {
        /*  see http://code.google.com/p/bullet/issues/detail?id=349
            this margin check could could be added to other collision shapes too, or add some assert/warning somewhere  */
        val minDimension = halfExtents[halfExtents.minAxis()]
        setSafeMargin(minDimension, defaultMarginMultiplier)
    }

    /** getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version */
    override fun getAabb(trans: Transform, aabbMin: Vec3, aabbMax: Vec3) = getAabbSlow(trans, aabbMin, aabbMax)

    override fun getAabbSlow(trans: Transform, aabbMin: Vec3, aabbMax: Vec3) {
        //use localGetSupportingVertexWithoutMargin?
        for (i in 0..2) {
            val vec = Vec3()
            vec[i] = 1f

            val sv = localGetSupportingVertex(vec * trans.basis)

            var tmp = trans * sv
            aabbMax[i] = tmp[i] + margin
            vec[i] = -1f
            tmp = trans * localGetSupportingVertex(vec * trans.basis)
            aabbMin[i] = tmp[i] - margin
        }
    }

    val localScalingNV get() = localScaling

    override var margin
        get() = collisionMargin
        set(value) {
            collisionMargin = value
        }

    val marginNV get() = collisionMargin

    override val numPreferredPenetrationDirections = 0

    override fun getPreferredPenetrationDirection(index: Int, penetrationVector: Vec3) {
        throw Error()
    }
}