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
import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import bullet.linearMath.times
import bullet.linearMath.transformAabb
import kotlin.math.sqrt
import bullet.collision.broadphaseCollision.BroadphaseNativeTypes as BNT

/** The CylinderShape class implements a cylinder shape primitive, centered around the origin. Its central axis aligned
 *  with the Y axis. CylinderShapeX is aligned with the X axis and CylinderShapeZ around the Z axis.    */
open class CylinderShape(halfExtents: Vec3) : ConvexInternalShape() {

    var upAxis = 1
        protected set

    init {
        implicitShapeDimensions put (halfExtents * localScaling) - margin
        setSafeMargin(halfExtents)
        shapeType = BNT.CYLINDER_SHAPE_PROXYTYPE
    }

    fun getHalfExtentsWithMargin(): Vec3 {
        val halfExtents = getHalfExtentsWithoutMargin()
        halfExtents += margin
        return halfExtents
    }

    /** changed in Bullet 2.63: assume the scaling and margin are included  */
    fun getHalfExtentsWithoutMargin() = implicitShapeDimensions

    override fun getAabb(trans: Transform, aabbMin: Vec3, aabbMax: Vec3) =
            transformAabb(getHalfExtentsWithoutMargin(), margin, trans, aabbMin, aabbMax)

    override fun calculateLocalInertia(mass: Float, inertia: Vec3) {
        /*  cylinder is defined as following:
                - principle axis aligned along y by default, radius in x, z-value not used
                - for btCylinderShapeX: principle axis aligned along x, radius in y direction, z-value not used
                - for btCylinderShapeZ: principle axis aligned along z, radius in x direction, y-value not used */
        val halfExtents = getHalfExtentsWithMargin()    // get cylinder dimension
        val div12 = mass / 12f
        val div4 = mass / 4f
        val div2 = mass / 2f
        val idxRadius: Int
        val idxHeight: Int

        when (upAxis) { // get indices of radius and height of cylinder
            0 -> {      // cylinder is aligned along x
                idxRadius = 1; idxHeight = 0; }
            2 -> {      // cylinder is aligned along z
                idxRadius = 0; idxHeight = 2; }
            else -> {   // cylinder is aligned along y
                idxRadius = 0; idxHeight = 1; }
        }
        // calculate cylinder radius & height squares
        val radius2 = halfExtents[idxRadius] * halfExtents[idxRadius]
        val height2 = 4f * halfExtents[idxHeight] * halfExtents[idxHeight]

        // calculate tensor terms
        val t1 = div12 * height2 + div4 * radius2
        val t2 = div2 * radius2

        when (upAxis) {  // set diagonal elements of inertia tensor
            0 -> inertia.put(t2, t1, t1)    // cylinder is aligned along x
            2 -> inertia.put(t1, t1, t2)    // cylinder is aligned along z
            else -> inertia.put(t1, t2, t1) // cylinder is aligned along y
        }
    }

    override fun localGetSupportingVertexWithoutMargin(vec: Vec3) = cylinderLocalSupportY(getHalfExtentsWithoutMargin(), vec)

    override fun batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: Array<Vec3>, supportVerticesOut: Array<Vec3>, numVectors: Int) {
        for (i in 0 until numVectors)
            supportVerticesOut[i] = cylinderLocalSupportY(getHalfExtentsWithoutMargin(), vectors[i])
    }

    override var margin
        get() = super.margin
        set(value) {
            //correct the m_implicitShapeDimensions for the margin
            val implicitShapeDimensionsWithMargin = implicitShapeDimensions + margin
            super.margin = collisionMargin
            implicitShapeDimensions put implicitShapeDimensionsWithMargin - margin
        }

    override fun localGetSupportingVertex(vec: Vec3): Vec3 {

        val supVertex = Vec3()
        supVertex put localGetSupportingVertexWithoutMargin(vec)

        if (margin != 0f) {
            val vecnorm = Vec3(vec)
            if (vecnorm.length2() < Float.EPSILON * Float.EPSILON)
                vecnorm put -1f
            vecnorm.normalize()
            supVertex += margin * vecnorm
        }
        return supVertex
    }

    override val anisotropicRollingFrictionDirection get() = Vec3().apply { set(upAxis, 1f) }

    open val radius get() = getHalfExtentsWithMargin().x

    override var localScaling
        get() = super.localScaling
        set(value) {
            val implicitShapeDimensionsWithMargin = implicitShapeDimensions + margin
            val unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / localScaling
            super.localScaling = value
            implicitShapeDimensions put (unScaledImplicitShapeDimensionsWithMargin * localScaling) - margin
        }

    override val name get() = "CylinderY"

    private fun cylinderLocalSupportY(halfExtents: Vec3, v: Vec3): Vec3 {
        val cylinderUpAxis = 1
        val xx = 0
        val yy = 1
        val zz = 2

        val radius = halfExtents[xx]
        val halfHeight = halfExtents[cylinderUpAxis]

        val tmp = Vec3()

        val s = sqrt(v[xx] * v[xx] + v[zz] * v[zz])
        return if (s != 0f) {
            val d = radius / s
            tmp[xx] = v[xx] * d
            tmp[yy] = if (v[yy] < 0f) -halfHeight else halfHeight
            tmp[zz] = v[zz] * d
            tmp
        } else {
            tmp[xx] = radius
            tmp[yy] = if (v[yy] < 0f) -halfHeight else halfHeight
            tmp[zz] = 0f
            tmp
        }
    }
}

class CylinderShapeX(halfExtents: Vec3) : CylinderShape(halfExtents) {
    init {
        upAxis = 0
    }

    override fun localGetSupportingVertexWithoutMargin(vec: Vec3) = cylinderLocalSupportX(getHalfExtentsWithoutMargin(), vec)
    override fun batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: Array<Vec3>, supportVerticesOut: Array<Vec3>, numVectors: Int) {
        for (i in 0 until numVectors)
            supportVerticesOut[i] = cylinderLocalSupportX(getHalfExtentsWithoutMargin(), vectors[i])
    }

    override val name get() = "CylinderX"

    override val radius get() = getHalfExtentsWithMargin().y

    private fun cylinderLocalSupportX(halfExtents: Vec3, v: Vec3): Vec3 {
        val cylinderUpAxis = 0
        val xx = 1
        val yy = 0
        val zz = 2

        //mapping depends on how cylinder local orientation is
        // extents of the cylinder is: X,Y is for radius, and Z for height

        val radius = halfExtents[xx]
        val halfHeight = halfExtents[cylinderUpAxis]

        val tmp = Vec3()

        val s = sqrt(v[xx] * v[xx] + v[zz] * v[zz])
        return if (s != 0f) {
            val d = radius / s
            tmp[xx] = v[xx] * d
            tmp[yy] = if (v[yy] < 0f) -halfHeight else halfHeight
            tmp[zz] = v[zz] * d
            tmp
        } else {
            tmp[xx] = radius
            tmp[yy] = if (v[yy] < 0f) -halfHeight else halfHeight
            tmp[zz] = 0f
            tmp
        }
    }
}

class CylinderShapeZ(halfExtents: Vec3) : CylinderShape(halfExtents) {
    init {
        upAxis = 2
    }

    override fun localGetSupportingVertexWithoutMargin(vec: Vec3) = cylinderLocalSupportZ(getHalfExtentsWithoutMargin(), vec)
    override fun batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: Array<Vec3>, supportVerticesOut: Array<Vec3>, numVectors: Int) {
        for (i in 0 until numVectors)
            supportVerticesOut[i] = cylinderLocalSupportZ(getHalfExtentsWithoutMargin(), vectors[i])
    }

    override val name get() = "CylinderZ"

    override val radius get() = getHalfExtentsWithMargin().x

    private fun cylinderLocalSupportZ(halfExtents: Vec3, v: Vec3): Vec3 {
        val cylinderUpAxis = 2
        val xx = 0
        val yy = 2
        val zz = 1

        //mapping depends on how cylinder local orientation is
        // extents of the cylinder is: X,Y is for radius, and Z for height

        val radius = halfExtents[xx]
        val halfHeight = halfExtents[cylinderUpAxis]

        val tmp = Vec3()

        val s = sqrt(v[xx] * v[xx] + v[zz] * v[zz])
        return if (s != 0f) {
            val d = radius / s
            tmp[xx] = v[xx] * d
            tmp[yy] = if (v[yy] < 0f) -halfHeight else halfHeight
            tmp[zz] = v[zz] * d
            tmp
        } else {
            tmp[xx] = radius
            tmp[yy] = if (v[yy] < 0f) -halfHeight else halfHeight
            tmp[zz] = 0f
            tmp
        }
    }
}