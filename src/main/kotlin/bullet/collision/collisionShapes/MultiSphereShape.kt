package bullet.collision.collisionShapes

import bullet.EPSILON
import bullet.linearMath.LARGE_FLOAT
import bullet.linearMath.Vec3
import kotlin.math.min
import kotlin.math.sqrt
import bullet.collision.broadphaseCollision.BroadphaseNativeTypes as BNT

/** The btMultiSphereShape represents the convex hull of a collection of spheres. You can create special capsules or
 *  other smooth volumes.
 *  It is possible to animate the spheres for deformation, but call 'recalcLocalAabb' after changing
 *  any sphere position/radius  */
class MultiSphereShape : ConvexInternalAabbCachingShape {

    var localPositionArray: Array<Vec3>
    var radiArray: FloatArray

    constructor(positions: Array<Vec3>, radi: FloatArray, numSpheres: Int) {
        localPositionArray = positions
        radiArray = radi
        shapeType = BNT.MULTI_SPHERE_SHAPE_PROXYTYPE
        //btScalar startMargin = btScalar(BT_LARGE_FLOAT);
        recalcLocalAabb()
    }

    constructor(position: Vec3, radi: Float, numSpheres: Int) : this(arrayOf(position), floatArrayOf(radi), numSpheres)

    /** CollisionShape Interface    */
    override fun calculateLocalInertia(mass: Float, inertia: Vec3) {
        //as an approximation, take the inertia of the box that bounds the spheres
        val localAabbMin = Vec3()
        val localAabbMax = Vec3()
        getCachedLocalAabb(localAabbMin, localAabbMax)
        val halfExtents = (localAabbMax - localAabbMin) * 0.5f

        val lx = 2f * halfExtents.x
        val ly = 2f * halfExtents.y
        val lz = 2f * halfExtents.z

        inertia.put(mass / 12f * (ly * ly + lz * lz),
                mass / 12f * (lx * lx + lz * lz),
                mass / 12f * (lx * lx + ly * ly))
    }

    /** btConvexShape Interface */
    override fun localGetSupportingVertexWithoutMargin(vec: Vec3): Vec3 {

        val supVec = Vec3()

        var maxDot = -LARGE_FLOAT

        val vec = Vec3(vec)
        val lenSqr = vec.length2()
        if (lenSqr < Float.EPSILON * Float.EPSILON)
            vec.put(1f, 0f, 0f)
        else
            vec *= 1f / sqrt(lenSqr)   // rlen

        var newDot: Float

        var pos = 0
        var rad = 0
        val numSpheres = localPositionArray.size
        val p = FloatArray(1)

        for (k in 0 until numSpheres step 128) {
            val temp = Array(128, { Vec3() })
            val innerCount = min(numSpheres - k, 128)
            for (i in 0 until innerCount) {
                temp[i] = localPositionArray[pos] * localScaling + vec * localScaling * radiArray[rad] - vec * margin
                pos++
                rad++
            }
            val i = vec.maxDot(temp, innerCount, p)
            newDot = p[0]
            if (newDot > maxDot) {
                maxDot = newDot
                supVec put temp[i]
            }
        }
        return supVec
    }

    override fun batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: Array<Vec3>,
                                                                   supportVerticesOut: Array<Vec3>, numVectors: Int) {
        for (j in 0 until numVectors) {

            var maxDot = -LARGE_FLOAT

            val vec = vectors[j]

            var newDot: Float

            var pos = 0
            var rad = 0
            val numSpheres = localPositionArray.size
            val p = FloatArray(1)

            for (k in 0 until numSpheres step 128) {
                val temp = Array(128, { Vec3() })
                val innerCount = min(numSpheres - k, 128)
                for (i in 0 until innerCount) {
                    temp[i] = localPositionArray[pos] * localScaling + vec * localScaling * radiArray[rad] - vec * margin
                    pos++
                    rad++
                }
                val i = vec.maxDot(temp, innerCount, p)
                newDot = p[0]
                if (newDot > maxDot) {
                    maxDot = newDot
                    supportVerticesOut[j] = temp[i]
                }
            }
        }
    }

    val sphereCount get() = localPositionArray.size

    fun getSpherePosition(index: Int) = localPositionArray[index]

    fun getSphereRadius(index: Int) = radiArray[index]

    override val name get() = "MultiSphere"
}