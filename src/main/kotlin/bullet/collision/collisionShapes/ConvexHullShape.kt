/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

package bullet.collision.collisionShapes

import bullet.EPSILON
import bullet.linearMath.*
import bullet.collision.broadphaseCollision.BroadphaseNativeTypes as BNT

/** The ConvexHullShape implements an implicit convex hull of an array of vertices.
 *  Bullet provides a general and fast collision detector for convex shapes based on GJK and EPA using localGetSupportingVertex.    */
class ConvexHullShape
/** this constructor optionally takes in a pointer to points. Each point is assumed to be 3 consecutive float (x,y,z),
 *  the striding defines the number of bytes between each point, in memory.
 *  It is easier to not pass any points in the constructor, and just add one point at a time, using addPoint.
 *  ConvexHullShape make an internal copy of the points.    */
(points: FloatArray, stride: Int = Vec3.size) : PolyhedralConvexAabbCachingShape() {

    var unscaledPoints = arrayOf<Vec3>()

    init {
        shapeType = BNT.CONVEX_HULL_SHAPE_PROXYTYPE
        unscaledPoints = Array(points.size, { Vec3(points, it * stride) })
        recalcLocalAabb()
    }

    fun addPoint(point: Vec3, recalculateLocalAabb: Boolean = true) {
        val size = unscaledPoints.size
        unscaledPoints = Array(size + 1, { if (it != size) unscaledPoints[it] else point })
        if (recalculateLocalAabb) recalcLocalAabb()
    }

    fun optimizeConvexHull() {
        val conv = ConvexHullComputer()
        TODO()
//        conv.compute(unscaledPoints, sizeof(btVector3),m_unscaledPoints.size(),0.f,0.f);
//        int numVerts = conv.vertices.size();
//        m_unscaledPoints.resize(0);
//        for (int i=0;i<numVerts;i++)
//            m_unscaledPoints.push_back(conv.vertices[i])
    }

    fun getScaledPoint(i: Int) = unscaledPoints[i] * localScaling

    val numPoints get() = unscaledPoints.size

    override fun localGetSupportingVertex(vec: Vec3): Vec3 {
        val supVertex = localGetSupportingVertexWithoutMargin(vec)

        if (margin != 0f) {
            val vecNorm = Vec3(vec)
            if (vecNorm.length2() < Float.EPSILON * Float.EPSILON)
                vecNorm put -1f
            vecNorm.normalize()
            supVertex += margin * vecNorm
        }
        return supVertex
    }

    override fun localGetSupportingVertexWithoutMargin(vec: Vec3): Vec3 {
        val supVec = Vec3(0f)
        var maxDot = -LARGE_FLOAT

        // Here we take advantage of dot(a, b*c) = dot(a*b, c).  Note: This is true mathematically, but not numerically.
        if (unscaledPoints.isNotEmpty()) {
            val scaled = vec * localScaling
            val p = FloatArray(1)
            val index = scaled.maxDot(unscaledPoints, unscaledPoints.size, p) // FIXME: may violate encapsulation of m_unscaledPoints
            maxDot = p[0]
            return unscaledPoints[index] * localScaling
        }
        return supVec
    }

    override fun batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: Array<Vec3>, supportVerticesOut: Array<Vec3>, numVectors: Int) {
        //use 'w' component of supportVerticesOut?
        for (i in 0 until numVectors)
            supportVerticesOut[i][3] = -LARGE_FLOAT

        for (j in 0 until numVectors) {
            val vec = vectors[j] * localScaling        // dot(a*b,c) = dot(a,b*c)
            if (unscaledPoints.isNotEmpty()) {
                val newDot = FloatArray(1)
                val i = vec.maxDot(unscaledPoints, unscaledPoints.size, newDot)
                supportVerticesOut[j] = getScaledPoint(i)
                supportVerticesOut[j][3] = newDot[0]
            } else
                supportVerticesOut[j][3] = -LARGE_FLOAT
        }
    }


    override fun project(trans: Transform, dir: Vec3, proj: FloatArray, witnesPtMin: Vec3, witnesPtMax: Vec3) {
        val min = 0
        val max = 1
        proj[min] = Float.MAX_VALUE
        proj[max] = -Float.MAX_VALUE

        val numVerts = unscaledPoints.size
        for (i in 0 until numVerts) {
            val vtx = unscaledPoints[i] * localScaling
            val pt = trans(vtx)
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
    }

    /** debugging   */
    override val name get() = "Convex"

    /** currently just for debugging (drawing), perhaps future support for algebraic continuous collision detection
     *  Please note that you can debug-draw btConvexHullShape with the Raytracer Demo   */
    override val numVertices get() = unscaledPoints.size
    override val numEdges get() = unscaledPoints.size
    override fun getEdge(i: Int, pa: Vec3, pb: Vec3) {
        val index0 = i % unscaledPoints.size
        val index1 = (i + 1) % unscaledPoints.size
        pa put getScaledPoint(index0)
        pb put getScaledPoint(index1)
    }

    override fun getVertex(i: Int, vtx: Vec3) = vtx.put(getScaledPoint(i))
    override val numPlanes get() = 0
    override fun getPlane(planeNormal:Vec3,planeSupport:Vec3,i:Int ) = throw Error()
    override fun isInside(pt:Vec3,tolerance:Float) = throw Error()

    /** in case we receive negative scaling */
    override var localScaling: Vec3
        get() = super.localScaling
        set(value) {
            localScaling put value
            recalcLocalAabb()
        }
}