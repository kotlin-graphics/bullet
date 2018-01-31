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

import bullet.linearMath.*
import bullet.collision.broadphaseCollision.BroadphaseNativeTypes as BNT

/** The BoxShape is a box primitive around the origin, its sides axis aligned with length specified by half extents, in
 *  local shape coordinates. When used as part of a CollisionObject or RigidBody it will be an oriented box in world space. */
class BoxShape(boxHalfExtents: Vec3) : PolyhedralConvexShape() {

    init {
        shapeType = BNT.BOX_SHAPE_PROXYTYPE
        implicitShapeDimensions put (boxHalfExtents * localScaling) - margin
        setSafeMargin(boxHalfExtents)
    }

    fun getHalfExtentsWithMargin(): Vec3 {
        val halfExtents = getHalfExtentsWithoutMargin()
        halfExtents += margin
        return halfExtents
    }

    /** scaling is included, margin is not  */
    fun getHalfExtentsWithoutMargin() = implicitShapeDimensions

    override fun localGetSupportingVertex(vec: Vec3): Vec3 {
        val halfExtents = getHalfExtentsWithoutMargin()
        halfExtents += margin
        return Vec3(fsels(vec.x, halfExtents.x, -halfExtents.x),
                fsels(vec.y, halfExtents.y, -halfExtents.y),
                fsels(vec.z, halfExtents.z, -halfExtents.z))
    }

    override fun localGetSupportingVertexWithoutMargin(vec: Vec3): Vec3 {
        val halfExtents = getHalfExtentsWithoutMargin()
        return Vec3(fsels(vec.x, halfExtents.x, -halfExtents.x),
                fsels(vec.y, halfExtents.y, -halfExtents.y),
                fsels(vec.z, halfExtents.z, -halfExtents.z))
    }

    override fun batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: Array<Vec3>, supportVerticesOut: Array<Vec3>, numVectors: Int) {
        val halfExtents = getHalfExtentsWithoutMargin()
        vectors.forEachIndexed { i, vec ->
            supportVerticesOut[i].put(fsels(vec.x, halfExtents.x, -halfExtents.x),
                    fsels(vec.y, halfExtents.y, -halfExtents.y),
                    fsels(vec.z, halfExtents.z, -halfExtents.z))
        }
    }


//    btBoxShape( const btVector3& boxHalfExtents)

    override var margin
        get() = super.margin
        set(value) {
            //correct the m_implicitShapeDimensions for the margin
            val implicitShapeDimensionsWithMargin = implicitShapeDimensions + margin
            collisionMargin = value
            implicitShapeDimensions put (implicitShapeDimensionsWithMargin - value)
        }

    override var localScaling
        get() = super.localScaling
        set(value) {
            val implicitShapeDimensionsWithMargin = implicitShapeDimensions + margin
            val unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / localScaling

            super.localScaling = value

            implicitShapeDimensions put (unScaledImplicitShapeDimensionsWithMargin * localScaling) - margin
        }

    override fun getAabb(trans: Transform, aabbMin: Vec3, aabbMax: Vec3) =
            transformAabb(getHalfExtentsWithoutMargin(), margin, trans, aabbMin, aabbMax)

    override fun calculateLocalInertia(mass: Float, inertia: Vec3) {
        //btScalar margin = btScalar(0.);
        val halfExtents = getHalfExtentsWithMargin()

        val lx = 2f * halfExtents.x
        val ly = 2f * halfExtents.y
        val lz = 2f * halfExtents.z

        inertia.put(mass / 12f * (ly * ly + lz * lz), mass / 12f * (lx * lx + lz * lz), mass / 12f * (lx * lx + ly * ly))
    }

    override fun getPlane(planeNormal: Vec3, planeSupport: Vec3, i: Int) {
        //this plane might not be aligned...
        val plane = Vec4()
        getPlaneEquation(plane, i)
        planeNormal put plane
        planeSupport put localGetSupportingVertex(-planeNormal)
    }

    override val numPlanes get() = 6
    override val numVertices get() = 8
    override val numEdges get() = 12

    override fun getVertex(i: Int, vtx: Vec3) {
        val halfExtents = getHalfExtentsWithMargin()

        vtx.put(halfExtents.x * (1 - (i and 1)) - halfExtents.x * (i and 1),
                halfExtents.y * (1 - ((i and 2) ushr 1)) - halfExtents.y * ((i and 2) ushr 1),
                halfExtents.z * (1 - ((i and 4) ushr 2)) - halfExtents.z * ((i and 4) ushr 2))
    }

    fun getPlaneEquation(plane: Vec4, i: Int) {
        val halfExtents = getHalfExtentsWithoutMargin()

        when (i) {
            0 -> plane.put(1f, 0f, 0f, -halfExtents.x)
            1 -> plane.put(-1f, 0f, 0f, -halfExtents.x)
            2 -> plane.put(0f, 1f, 0f, -halfExtents.y)
            3 -> plane.put(0f, -1f, 0f, -halfExtents.y)
            4 -> plane.put(0f, 0f, 1f, -halfExtents.z)
            5 -> plane.put(0f, 0f, -1f, -halfExtents.z)
            else -> throw Error()
        }
    }

    override fun getEdge(i: Int, pa: Vec3, pb: Vec3) {  //virtual void getEdge(int i,Edge& edge) const
        val edgeVert0: Int
        val edgeVert1: Int
        when (i) {
            0 -> {
                edgeVert0 = 0; edgeVert1 = 1; }
            1 -> {
                edgeVert0 = 0; edgeVert1 = 2; }
            2 -> {
                edgeVert0 = 1; edgeVert1 = 3; }
            3 -> {
                edgeVert0 = 2; edgeVert1 = 3; }
            4 -> {
                edgeVert0 = 0; edgeVert1 = 4; }
            5 -> {
                edgeVert0 = 1; edgeVert1 = 5; }
            6 -> {
                edgeVert0 = 2; edgeVert1 = 6; }
            7 -> {
                edgeVert0 = 3; edgeVert1 = 7; }
            8 -> {
                edgeVert0 = 4; edgeVert1 = 5; }
            9 -> {
                edgeVert0 = 4; edgeVert1 = 6; }
            10 -> {
                edgeVert0 = 5; edgeVert1 = 7; }
            11 -> {
                edgeVert0 = 6; edgeVert1 = 7; }
            else -> throw Error()
        }
        getVertex(edgeVert0, pa)
        getVertex(edgeVert1, pb)
    }

    override fun isInside(pt: Vec3, tolerance: Float): Boolean {
        val halfExtents = getHalfExtentsWithoutMargin()
        //btScalar minDist = 2*tolerance;
        return (pt.x <= (halfExtents.x + tolerance)) && (pt.x >= (-halfExtents.x - tolerance)) &&
                (pt.y <= (halfExtents.y + tolerance)) && (pt.y >= (-halfExtents.y - tolerance)) &&
                (pt.z <= (halfExtents.z + tolerance)) && (pt.z >= (-halfExtents.z - tolerance))
    }

    override val name get() = "Box"

    override val numPreferredPenetrationDirections get() = 6

    override fun getPreferredPenetrationDirection(index: Int, penetrationVector: Vec3) {
        when (index) {
            0 -> penetrationVector.put(1f, 0f, 0f)
            1 -> penetrationVector.put(-1f, 0f, 0f)
            2 -> penetrationVector.put(0f, 1f, 0f)
            3 -> penetrationVector.put(0f, -1f, 0f)
            4 -> penetrationVector.put(0f, 0f, 1f)
            5 -> penetrationVector.put(0f, 0f, -1f)
            else -> throw Error()
        }
    }
}