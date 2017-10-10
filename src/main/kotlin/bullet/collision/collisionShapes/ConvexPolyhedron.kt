package bullet.collision.collisionShapes

import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import bullet.linearMath.times
import kotlin.math.abs
import kotlin.math.sqrt

class ConvexPolyhedron {

    lateinit var vertices: Array<Vec3>
    lateinit var faces: Array<Face>
    lateinit var uniqueEdges: ArrayList<Vec3>

    val localCenter = Vec3()
    var extents = Vec3()
    var radius = 0f
    var c = Vec3()
    var e = Vec3()

    val Vec3.isAlmostZero get() = abs(x) <= 1e-6f && abs(y) <= 1e-6f && abs(z) <= 1e-6

    fun initialize() {

        val edges = HashMap<InternalVertexPair, InternalEdge>()

        var totalArea = 0f

        localCenter put 0f
        faces.forEachIndexed { i, face ->
            val numVertices = face.indices.size
            val nbTris = numVertices
            for (j in 0 until nbTris) {
                val k = (j + 1) % numVertices
                val vp = InternalVertexPair(face.indices[j], face.indices[k])
                val editr = edges[vp]
                val edge = vertices[vp.v1] - vertices[vp.v0]
                edge.normalize()

                var found = false

                for (p in 0 until uniqueEdges.size) {

                    if ((uniqueEdges[p] - edge).isAlmostZero || (uniqueEdges[p] + edge).isAlmostZero) {
                        found = true
                        break
                    }
                }

                if (!found) uniqueEdges.add(edge)

                if (editr != null) {
                    assert(editr.face0 >= 0)
                    assert(editr.face1 < 0)
                    editr.face1 = i
                } else
                    edges[vp] = InternalEdge(face0 = i)
            }
        }

        faces.forEach {
            val numVertices = it.indices.size
            val nbTris = numVertices - 2

            val p0 = vertices[it.indices[0]]
            for (j in 1..nbTris) {
                val k = (j + 1) % numVertices
                val p1 = vertices[it.indices[j]]
                val p2 = vertices[it.indices[k]]
                val area = ((p0 - p1) cross (p0 - p2)).length() * 0.5f
                val center = (p0 + p1 + p2) / 3f
                localCenter += area * center
                totalArea += area
            }
        }
        localCenter /= totalArea

        radius = Float.MAX_VALUE
        faces.forEach {
            val normal = Vec3(it.plane)
            val dist = abs(localCenter dot normal + it.plane[3]) // TODO check priority
            if (dist < radius) radius = dist
        }

        var minX = Float.MAX_VALUE
        var minY = Float.MAX_VALUE
        var minZ = Float.MAX_VALUE
        var maxX = -Float.MAX_VALUE
        var maxY = -Float.MAX_VALUE
        var maxZ = -Float.MAX_VALUE
        vertices.forEach {
            if (it.x < minX) minX = it.x
            if (it.x > maxX) maxX = it.x
            if (it.y < minY) minY = it.y
            if (it.y > maxY) maxY = it.y
            if (it.z < minZ) minZ = it.z
            if (it.z > maxZ) maxZ = it.z
        }
        c.put(maxX + minX, maxY + minY, maxZ + minZ)
        e.put(maxX - minX, maxY - minY, maxZ - minZ)


//		const btScalar r = m_radius / sqrtf(2.0f);
        val r = radius / sqrt(3f)
        val largestExtent = e.maxAxis()
        var step = (e[largestExtent] * 0.5f - r) / 1024f
        extents put r
        extents[largestExtent] = e[largestExtent] * 0.5f
        var foundBox = false
        for (j in 0 until 1024) {
            if (testContainment()) {
                foundBox = true
                break
            }
            extents[largestExtent] -= step
        }
        if (!foundBox) extents put r
        else {
            // Refine the box
            step = (radius - r) / 1024f
            val e0 = (1 shl largestExtent) and 3
            val e1 = (1 shl e0) and 3

            for (j in 0 until 1024) {
                val saved0 = extents[e0]
                val saved1 = extents[e1]
                extents[e0] += step
                extents[e1] += step

                if (!testContainment()) {
                    extents[e0] = saved0
                    extents[e1] = saved1
                    break
                }
            }
        }
    }

    fun testContainment(): Boolean {
        for (p in 0..7) {
            val localPt = when (p) {
                0 -> localCenter + Vec3(extents[0])
                1 -> localCenter + Vec3(extents[0], extents[1], -extents[2])
                2 -> localCenter + Vec3(extents[0], -extents[1], extents[2])
                3 -> localCenter + Vec3(extents[0], -extents[1], -extents[2])
                4 -> localCenter + Vec3(-extents[0], extents[1], extents[2])
                5 -> localCenter + Vec3(-extents[0], extents[1], -extents[2])
                6 -> localCenter + Vec3(-extents[0], -extents[1], extents[2])
                7 -> localCenter + Vec3(-extents[0], -extents[1], -extents[2])
                else -> throw Error()
            }

            faces.forEach {
                val normal = Vec3(it.plane)
                val d = localPt dot normal + it.plane[3]
                if (d > 0f) return false
            }
        }
        return true
    }

    fun project(trans: Transform, dir: Vec3, proj: FloatArray, witnesPtMin: Vec3, witnesPtMax: Vec3) {
        val min = 0
        val max = 1
        proj[min] = Float.MAX_VALUE
        proj[max] = -Float.MAX_VALUE
        val numVerts = vertices.size
        for (i in 0 until numVerts) {
            val pt = trans * vertices[i]
            val dp = pt dot dir
            if (dp < proj[min]) {
                proj[min] = dp
                witnesPtMin put pt
            }
            if (dp > proj[max]) {
                proj[max] = dp
                witnesPtMax put pt
            }
        }
        if (proj[min] > proj[max]) {
            var tmp = proj[min]
            proj[min] = proj[max]
            proj[max] = tmp
            for(i in 0..2) {
                tmp = witnesPtMin[i]
                witnesPtMin[i] = witnesPtMax[i]
                witnesPtMax[i] = tmp
            }
        }
    }
}

class Face {
    lateinit var indices: IntArray
    //	btAlignedObjectArray<int>	m_connectedFaces;
    val plane = FloatArray(4)
}

class InternalVertexPair(var v0: Int, var v1: Int) {
    init {
        if (v1 > v0) {
            val tmp = v0
            v0 = v1
            v1 = tmp
        }
    }

    val hash get() = v0 + (v1 shl 16)
    override fun equals(other: Any?) = other is InternalVertexPair && v0 == other.v0 && v1 == other.v0
    override fun hashCode() = 31 * v0.hashCode() + v1.hashCode()
}

class InternalEdge(var face0: Int = -1, var face1: Int = -1)