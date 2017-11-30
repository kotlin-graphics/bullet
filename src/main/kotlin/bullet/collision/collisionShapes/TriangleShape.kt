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

import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import bullet.collision.broadphaseCollision.BroadphaseNativeTypes as BNT

class TriangleShape : PolyhedralConvexShape {

    val vertices1 = Array(3, { Vec3() })

    override val numVertices get() = 3

    fun getVertexPtr(index: Int) = vertices1[index]

    override fun getVertex(i: Int, vtx: Vec3) = vtx put vertices1[i]

    override val numEdges get() = 3

    override fun getEdge(i: Int, pa: Vec3, pb: Vec3) {
        getVertex(i, pa)
        getVertex((i + 1) % 3, pb)
    }

    override fun getAabb(trans: Transform, aabbMin: Vec3, aabbMax: Vec3) = getAabbSlow(trans, aabbMin, aabbMax)

    override fun localGetSupportingVertexWithoutMargin(vec: Vec3) = vertices1[(vec dot3 vertices1).maxAxis()]

    override fun batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: Array<Vec3>, supportVerticesOut: Array<Vec3>, numVectors: Int) {
        for (i in 0 until numVectors) {
            val dir = vectors[i]
            val dots = dir dot3 vertices1
            supportVerticesOut[i] = vertices1[dots.maxAxis()]
        }
    }

    constructor() : super() {
        shapeType = BNT.TRIANGLE_SHAPE_PROXYTYPE
    }

    constructor(p0: Vec3, p1: Vec3, p2: Vec3) : super() {
        shapeType = BNT.TRIANGLE_SHAPE_PROXYTYPE
        vertices1[0] put p0
        vertices1[1] put p1
        vertices1[2] put p2
    }

    override fun getPlane(planeNormal: Vec3, planeSupport: Vec3, i: Int) = getPlaneEquation(i, planeNormal, planeSupport)

    override val numPlanes get() = 1

    fun calcNormal(normal: Vec3 = Vec3()): Vec3 {
        normal put ((vertices1[1] - vertices1[0]) cross (vertices1[2] - vertices1[0]))
        return normal.normalize()
    }

    fun getPlaneEquation(i: Int, planeNormal: Vec3, planeSupport: Vec3) {
        calcNormal(planeNormal)
        planeSupport put vertices1[0]
    }

    override fun calculateLocalInertia(mass: Float, inertia: Vec3) {
        TODO()
//                btAssert(0)
//        inertia.setValue(btScalar(0.), btScalar(0.), btScalar(0.))
    }

    override fun isInside(pt: Vec3, tolerance: Float): Boolean {
        val normal = calcNormal()
        //distance to plane
        var dist = pt dot normal
        val planeConst = vertices1[0] dot normal
        dist -= planeConst
        if (dist in -tolerance..tolerance) {
            //inside check on edge-planes
            for (i in 0..2) {
                val pa = Vec3()
                val pb = Vec3()
                getEdge(i, pa, pb)
                val edge = pb - pa
                val edgeNormal = edge cross normal
                edgeNormal.normalize()
                var dist = pt dot edgeNormal
                val edgeConst = pa dot edgeNormal
                dist -= edgeConst
                if (dist < -tolerance) return false
            }
            return true
        }
        return false
    }

    override val name get() = "Triangle"

    override val numPreferredPenetrationDirections get() = 2

    override fun getPreferredPenetrationDirection(index: Int, penetrationVector: Vec3) {
        calcNormal(penetrationVector)
        if (index != 0) penetrationVector *= -1f
    }
}