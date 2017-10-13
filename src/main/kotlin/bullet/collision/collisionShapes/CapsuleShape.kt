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

import bullet.linearMath.LARGE_FLOAT
import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import kotlin.math.sqrt
import bullet.collision.broadphaseCollision.BroadphaseNativeTypes as BNT

/** The CapsuleShape represents a capsule around the Y axis, there is also the CapsuleShapeX aligned around the X axis
 *  and CapsuleShapeZ around the Z axis.
 *  The total height is height + 2 * radius, so the height is just the height between the center of each 'sphere'
 *  of the capsule caps.
 *  The CapsuleShape is a convex hull of two spheres. The MultiSphereShape is a more general collision shape that takes
 *  the convex hull of multiple sphere, so it can also represent a capsule when just using two spheres. */
open class CapsuleShape : ConvexInternalShape {

    var upAxis = 0
        protected set

    /** only used for btCapsuleShapeZ and btCapsuleShapeX subclasses.   */
    constructor() : super() {
        shapeType = BNT.CAPSULE_SHAPE_PROXYTYPE
    }

    constructor(radius: Float, height: Float) : super() {
        collisionMargin = radius
        shapeType = BNT.CAPSULE_SHAPE_PROXYTYPE
        upAxis = 1
        implicitShapeDimensions.put(radius, 0.5f * height, radius)
    }

    /** CollisionShape Interface    */
    override fun calculateLocalInertia(mass: Float, inertia: Vec3) {
        //as an approximation, take the inertia of the box that bounds the spheres
        val ident = Transform()
        ident.setIdentity()

        val halfExtents = Vec3(radius)
        halfExtents[upAxis] += halfHeight

        val lx = 2f * halfExtents[0]
        val ly = 2f * halfExtents[1]
        val lz = 2f * halfExtents[2]
        val x2 = lx * lx
        val y2 = ly * ly
        val z2 = lz * lz
        val scaledmass = mass * .08333333f

        inertia[0] = scaledmass * (y2 + z2)
        inertia[1] = scaledmass * (x2 + z2)
        inertia[2] = scaledmass * (x2 + y2)
    }

    /** ConvexShape Interface   */
    override fun localGetSupportingVertexWithoutMargin(vec: Vec3): Vec3 {

        val supVec = Vec3()

        var maxDot = -LARGE_FLOAT

        val vec = Vec3(vec)
        val lenSqr = vec.length2()
        if (lenSqr < 0.0001f)
            vec.put(1f, 0f, 0f)
        else
            vec *= 1f / sqrt(lenSqr)   // rlen

        val vtx = Vec3()
        var newDot: Float

        run {
            val pos = Vec3()
            pos[upAxis] = halfHeight

            vtx put pos
            newDot = vec dot vtx
            if (newDot > maxDot) {
                maxDot = newDot
                supVec put vtx
            }
        }
        run {
            val pos = Vec3()
            pos[upAxis] = -halfHeight

            vtx put pos
            newDot = vec dot vtx
            if (newDot > maxDot) {
                maxDot = newDot
                supVec put vtx
            }
        }
        return supVec
    }

    override fun batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: Array<Vec3>, supportVerticesOut: Array<Vec3>, numVectors: Int) {
        for (j in 0 until numVectors) {
            var maxDot = -LARGE_FLOAT
            val vec = vectors[j]

            val vtx = Vec3()
            var newDot: Float
            run {
                val pos = Vec3()
                pos[upAxis] = halfHeight
                vtx put pos
                newDot = vec dot vtx
                if (newDot > maxDot) {
                    maxDot = newDot
                    supportVerticesOut[j] = vtx
                }
            }
            run {
                val pos = Vec3()
                pos[upAxis] = -halfHeight
                vtx put pos
                newDot = vec dot vtx
                if (newDot > maxDot) {
                    maxDot = newDot
                    supportVerticesOut[j] = vtx
                }
            }
        }
    }

    //don't override the margin for capsules, their entire radius == margin
//    virtual void setMargin(btScalar collisionMargin)

    override fun getAabb(trans: Transform, aabbMin: Vec3, aabbMax: Vec3) {
        val halfExtents = Vec3(radius)
        halfExtents[upAxis] = radius + halfHeight
        val abs_b = trans.basis.absolute()
        val center = trans.origin
        val extent = halfExtents dot3 abs_b

        aabbMin put center - extent
        aabbMax put center + extent
    }

    override val name get() = "CapsuleShape"

    val radius get() = implicitShapeDimensions[(upAxis + 2) % 3]

    val halfHeight get() = implicitShapeDimensions[upAxis]

    override var localScaling
        get() = super.localScaling
        set(value) {
            val unScaledImplicitShapeDimensions = implicitShapeDimensions / localScaling
            super.localScaling = value
            implicitShapeDimensions put unScaledImplicitShapeDimensions * value
            //update m_collisionMargin, since entire radius==margin
            val radiusAxis = (upAxis + 2) % 3
            collisionMargin = implicitShapeDimensions[radiusAxis]
        }

    override val anisotropicRollingFrictionDirection get() = Vec3().apply { set(upAxis, 1f) }
}

/** CapsuleShapeX represents a capsule around the Z axis
 *  the total height is height + 2 * radius, so the height is just the height between the center of each 'sphere'
 *  of the capsule caps.    */
class CapsuleShapeX(radius: Float, height: Float) : CapsuleShape(radius, height) {
    /** debugging   */
    override val name get() = "CapsuleX"
}

/** CapsuleShapeZ represents a capsule around the Z axis
 *  the total height is height + 2 * radius, so the height is just the height between the center of each 'sphere'
 *  of the capsule caps.    */
class CapsuleShapeZ(radius: Float, height: Float) : CapsuleShape(radius, height) {
    /** debugging   */
    override val name get() = "CapsuleZ"
}