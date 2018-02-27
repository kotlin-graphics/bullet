package bullet.collision.collisionShapes

import bullet.EPSILON
import bullet.collision.broadphaseCollision.BroadphaseNativeTypes.*
import bullet.linearMath.*
import kotlin.math.sqrt

val MAX_PREFERRED_PENETRATION_DIRECTIONS = 10

abstract class ConvexShape : CollisionShape() {

    abstract fun localGetSupportingVertex(vec: Vec3): Vec3

    open fun localGetSupportingVertexWithoutMargin(vec: Vec3) = Vec3()

    open infix fun localGetSupportVertexWithoutMarginNonVirtual(localDir: Vec3): Vec3 = when (shapeType) {
        SPHERE_SHAPE_PROXYTYPE -> Vec3()
        BOX_SHAPE_PROXYTYPE -> {
            val convexShape = this as BoxShape
            val halfExtents = convexShape.implicitShapeDimensions
            Vec3(fsels(localDir.x, halfExtents.x, -halfExtents.x),
                    fsels(localDir.y, halfExtents.y, -halfExtents.y),
                    fsels(localDir.z, halfExtents.z, -halfExtents.z))
        }
        TRIANGLE_SHAPE_PROXYTYPE -> {
            val triangleShape = this as TriangleShape
            val dir = Vec3(localDir)
            val vertices = triangleShape.vertices
            val dots = dir.dot3(vertices[0], vertices[1], vertices[2])
            val sup = vertices[dots.maxAxis()]
            Vec3(sup.x, sup.y, sup.z)
        }
        CYLINDER_SHAPE_PROXYTYPE -> {
            val cylShape = this as CylinderShape
            // mapping of halfextents/dimension onto radius/height depends on how cylinder local orientation is (upAxis)
            val halfExtents = cylShape.implicitShapeDimensions
            val v = Vec3(localDir)
            val cylinderUpAxis = cylShape.upAxis
            val xx: Int
            val yy: Int
            val zz: Int
            when (cylinderUpAxis) {
                0 -> {
                    xx = 1; yy = 0; zz = 2; }
                1 -> {
                    xx = 0; yy = 1; zz = 2; }
                2 -> {
                    xx = 0; yy = 2; zz = 1; }
                else -> throw Error()
            }

            val radius = halfExtents[xx]
            val halfHeight = halfExtents[cylinderUpAxis]

            val tmp = Vec3()

            val s = sqrt(v[xx] * v[xx] + v[zz] * v[zz])
            if (s != 0f) {
                val d = radius / s
                tmp[xx] = v[xx] * d
                tmp[yy] = if (v[yy] < 0f) -halfHeight else halfHeight
                tmp[zz] = v[zz] * d
            } else {
                tmp[xx] = radius
                tmp[yy] = if (v[yy] < 0f) -halfHeight else halfHeight
                tmp[zz] = 0f
            }
            tmp
        }
        CAPSULE_SHAPE_PROXYTYPE -> {
            val vec0 = Vec3(localDir)

            val capsuleShape = this as CapsuleShape
            val halfHeight = capsuleShape.halfHeight
            val capsuleUpAxis = capsuleShape.upAxis

            val supVec = Vec3()

            var maxDot = -LARGE_FLOAT

            val vec = Vec3(vec0)
            val lenSqr = vec.length2()
            if (lenSqr < Float.EPSILON * Float.EPSILON)
                vec.put(1, 0, 0)
            else
                vec *= 1f / sqrt(lenSqr)
            lateinit var vtx: Vec3
            var newDot = 0f
            run {
                val pos = Vec3()
                pos[capsuleUpAxis] = halfHeight

                vtx = pos
                newDot = vec dot vtx


                if (newDot > maxDot) {
                    maxDot = newDot
                    supVec put vtx
                }
            }
            run {
                val pos = Vec3(0, 0, 0)
                pos[capsuleUpAxis] = -halfHeight

                vtx = pos
                newDot = vec.dot(vtx)
                if (newDot > maxDot) {
                    maxDot = newDot
                    supVec put vtx
                }
            }
            supVec
        }
        CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE -> {
            val convexPointCloudShape = this as ConvexPointCloudShape
            val points = convexPointCloudShape.unscaledPoints
            val numPoints = convexPointCloudShape.numPoints
            convexHullSupport(localDir, points, numPoints, convexPointCloudShape.localScalingNV)
        }
        CONVEX_HULL_SHAPE_PROXYTYPE -> {
            val convexHullShape = this as ConvexHullShape
            val points = convexHullShape.unscaledPoints
            val numPoints = convexHullShape.numPoints
            convexHullSupport(localDir, points, numPoints, convexHullShape.localScalingNV)
        }
        else -> localGetSupportingVertexWithoutMargin(localDir)
    }

    open fun localGetSupportVertexNonVirtual(localDir: Vec3): Vec3 {
        val localDirNorm = Vec3(localDir)
        if (localDirNorm.length2() < Float.EPSILON * Float.EPSILON)
            localDirNorm put -1f
        localDirNorm.normalize()

        return localGetSupportVertexWithoutMarginNonVirtual(localDirNorm) + getMarginNonVirtual() * localDirNorm
    }

    /* TODO: This should be bumped up to btCollisionShape () */
    open fun getMarginNonVirtual() = when (shapeType) {
        SPHERE_SHAPE_PROXYTYPE -> (this as SphereShape).radius
        BOX_SHAPE_PROXYTYPE -> TODO()//(this as BoxShape).getMarginNV()
        TRIANGLE_SHAPE_PROXYTYPE -> TODO()//(this as TriangleShape).getMarginNV()
        CYLINDER_SHAPE_PROXYTYPE -> TODO()//(this as CylinderShape).getMarginNV()
        CONE_SHAPE_PROXYTYPE -> TODO()//(this as ConeShape).getMarginNV()
        CAPSULE_SHAPE_PROXYTYPE -> TODO()//(this as CapsuleShape).getMarginNV()
        CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE, CONVEX_HULL_SHAPE_PROXYTYPE -> TODO()//(this as PolyhedralConvexShape).getMarginNV()
        else -> throw Error()   // should never reach here
    }

    open fun getAabbNonVirtual(t: Transform, aabbMin: Vec3, aabbMax: Vec3) = when (shapeType) {
        SPHERE_SHAPE_PROXYTYPE -> {
            val sphereShape = this as SphereShape
            val radius = sphereShape.implicitShapeDimensions.x  // * convexShape.getLocalScaling().x
            val margin = radius + sphereShape.getMarginNonVirtual()
            val center = t.origin
            val extent = Vec3(margin)
            aabbMin put (center - extent)
            aabbMax put (center + extent)
        }
        CYLINDER_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE -> {
            val convexShape = this as BoxShape
            val margin = convexShape.getMarginNonVirtual()
            val halfExtents = convexShape.implicitShapeDimensions
            halfExtents += Vec3(margin)
            val absB = t.basis.absolute()
            val center = t.origin
            val extent = halfExtents.dot3(absB[0], absB[1], absB[2])

            aabbMin put (center - extent)
            aabbMax put (center + extent)
        }
        TRIANGLE_SHAPE_PROXYTYPE -> {
            val triangleShape = this as TriangleShape
            val margin = triangleShape.getMarginNonVirtual()
            for (i in 0..2) {
                val vec = Vec3()
                vec[i] = 1f

                val sv = localGetSupportVertexWithoutMarginNonVirtual(vec * t.basis)

                var tmp = t * sv
                aabbMax[i] = tmp[i] + margin
                vec[i] = -1f
                tmp = t * localGetSupportVertexWithoutMarginNonVirtual(vec * t.basis)
                aabbMin[i] = tmp[i] - margin
            }
        }
        CAPSULE_SHAPE_PROXYTYPE -> {
            val capsuleShape = this as CapsuleShape
            val halfExtents = Vec3(capsuleShape.radius)
            val upAxis = capsuleShape.upAxis
            halfExtents[upAxis] = capsuleShape.radius + capsuleShape.halfHeight
            val absB = t.basis.absolute()
            val center = t.origin
            val extent = halfExtents.dot3(absB[0], absB[1], absB[2])
            aabbMin put (center - extent)
            aabbMax put (center + extent)
        }
        CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE, CONVEX_HULL_SHAPE_PROXYTYPE -> {
            val convexHullShape = this as PolyhedralConvexAabbCachingShape
            val margin = convexHullShape.getMarginNonVirtual()
            convexHullShape.getNonvirtualAabb(t, aabbMin, aabbMax, margin)
        }
        else -> getAabb(t, aabbMin, aabbMax)
    }


    open fun project(trans: Transform, dir: Vec3, minMax: FloatArray, witnesPtMin: Vec3, witnesPtMax: Vec3) {

        val localAxis = dir * trans.basis
        val vtx1 = trans * localGetSupportingVertex(localAxis)
        val vtx2 = trans * localGetSupportingVertex(-localAxis)

        minMax[0] = vtx1 dot dir
        minMax[1] = vtx2 dot dir
        witnesPtMax put vtx2
        witnesPtMin put vtx1

        if (minMax[0] > minMax[1]) {
            val tmp = minMax[0]
            minMax[0] = minMax[1]
            minMax[1] = tmp
            witnesPtMax put vtx1
            witnesPtMin put vtx2
        }
    }

    /** notice that the vectors should be unit length   */
    open fun batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: Array<Vec3>, supportVerticesOut: Array<Vec3>, numVectors: Int) {}

    /** getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version */
    override fun getAabb(trans: Transform, aabbMin: Vec3, aabbMax: Vec3) {}

    open fun getAabbSlow(trans: Transform, aabbMin: Vec3, aabbMax: Vec3) {}

    override var localScaling = Vec3()

    open val numPreferredPenetrationDirections get() = 0

    open fun getPreferredPenetrationDirection(index: Int, penetrationVector: Vec3) {}

    companion object {

        fun convexHullSupport(localDirOrg: Vec3, points: Array<Vec3>, numPoints: Int, localScaling: Vec3): Vec3 {

            val vec = localDirOrg * localScaling

            val p = FloatArray(1)
            var ptIndex = vec.maxDot(points, numPoints, p)
            var maxDot = p[0]
            assert(ptIndex >= 0)
            if(ptIndex < 0) ptIndex = 0
            return points[ptIndex] * localScaling   // supVec
        }
    }
}