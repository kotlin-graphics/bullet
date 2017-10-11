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
import bullet.linearMath.LARGE_FLOAT
import bullet.linearMath.Vec3
import bullet.linearMath.times
import kotlin.math.sqrt
import bullet.collision.broadphaseCollision.BroadphaseNativeTypes as BNT

/** The ConvexPointCloudShape implements an implicit convex hull of an array of vertices.   */
class ConvexPointCloudShape : PolyhedralConvexAabbCachingShape {

    var unscaledPoints = arrayOf<Vec3>()
    var numPoints = 0

    constructor() {
        localScaling put 1f
        shapeType = BNT.CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE
    }

    constructor(points: Array<Vec3>, numPoints: Int, localScaling: Vec3, computeAabb: Boolean = true) {
        this.localScaling put localScaling
        shapeType = BNT.CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE
        unscaledPoints = points
        this.numPoints = numPoints

        if (computeAabb) recalcLocalAabb()
    }

    fun setPoints(points: Array<Vec3>, numPoints: Int, computeAabb: Boolean = true, localScaling: Vec3 = Vec3(1f)) {
        unscaledPoints = points
        this.numPoints = numPoints
        this.localScaling = localScaling

        if (computeAabb) recalcLocalAabb()
    }

    fun getScaledPoint(index: Int) = unscaledPoints[index] * localScaling

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
        val supVec = Vec3()
        var maxDot = -LARGE_FLOAT

        val vec = Vec3(vec)
        val lenSqr = vec.length2()
        if (lenSqr < 0.0001f)
            vec.put(1f, 0f, 0f)
        else
            vec *= 1f / sqrt(lenSqr)    // rlen

        if (numPoints > 0) {
            /*  Here we take advantage of dot(a*b, c) = dot( a, b*c) to do less work. Note this transformation is true
                mathematically, not numerically.    */
            //    btVector3 scaled = vec * m_localScaling;
            val p = FloatArray(1)
            val index = vec.maxDot(unscaledPoints, numPoints, p)   //FIXME: may violate encapsulation of m_unscaledPoints
            maxDot = p[0]
            return getScaledPoint(index)
        }
        return supVec
    }

    override fun batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: Array<Vec3>, supportVerticesOut: Array<Vec3>, numVectors: Int) {
        for (j in 0 until numVectors) {
            val vec = vectors[j] * localScaling  // dot( a*c, b) = dot(a, b*c)
            val p = FloatArray(1)
            val index = vec.maxDot(unscaledPoints, numPoints, p)
            val maxDot = p[0]
            supportVerticesOut[j][3] = -LARGE_FLOAT
            if (0 <= index) {
                // WARNING: don't swap next lines, the w component would get overwritten!
                supportVerticesOut[j] = getScaledPoint(index)
                supportVerticesOut[j][3] = maxDot
            }
        }
    }

    /** debugging   */
    override val name get() = "ConvexPointCloud"

    /** currently just for debugging (drawing), perhaps future support for algebraic continuous collision detection
     *  Please note that you can debug-draw btConvexHullShape with the Raytracer Demo   */
    override val numVertices get() = numPoints

    override val numEdges get() = 0
    override fun getEdge(i: Int, pa: Vec3, pb: Vec3) = throw Error()
    override fun getVertex(i: Int, vtx: Vec3) = vtx.put(unscaledPoints[i] * localScaling)
    override val numPlanes get() = 0
    override fun getPlane(planeNormal: Vec3, planeSupport: Vec3, i: Int) = throw  Error()
    override fun isInside(pt:Vec3,tolerance:Float) = throw Error()

    override var localScaling
        get() = super.localScaling
        set(value) {
            localScaling put value
            recalcLocalAabb()
        }
}