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
import bullet.linearMath.times
import bullet.linearMath.planeSpace1

/** The StaticPlaneShape simulates an infinite non-moving (static) collision plane. */
class StaticPlaneShape(planeNormal: Vec3, val planeConstant: Float) : ConcaveShape() {

    val localAabbMin = Vec3()
    val localAabbMax = Vec3()

    val planeNormal = planeNormal.normalized()
    override var localScaling = Vec3(1f)

    override fun getAabb(trans: Transform, aabbMin: Vec3, aabbMax: Vec3) {
        aabbMin put -LARGE_FLOAT
        aabbMax put LARGE_FLOAT
    }

    override fun processAllTriangles(callback: TriangleCallback, aabbMin: Vec3, aabbMax: Vec3) {
        val halfExtents = (aabbMax - aabbMin) * 0.5f
        val radius = halfExtents.length()
        val center = (aabbMax + aabbMin) * 0.5f

        //this is where the triangles are generated, given AABB and plane equation (normal/constant)
        val tangentDir0 = Vec3()
        val tangentDir1 = Vec3()

        //tangentDir0/tangentDir1 can be precalculated
        planeSpace1(planeNormal, tangentDir0, tangentDir1)

        val projectedCenter = center - (planeNormal.dot(center) - planeConstant) * planeNormal

        val triangle = arrayOf(
                projectedCenter + tangentDir0 * radius + tangentDir1 * radius,
                projectedCenter + tangentDir0 * radius - tangentDir1 * radius,
                projectedCenter - tangentDir0 * radius - tangentDir1 * radius)

        callback.processTriangle(triangle, 0, 0)

        triangle[0] = projectedCenter - tangentDir0 * radius - tangentDir1 * radius
        triangle[1] = projectedCenter - tangentDir0 * radius + tangentDir1 * radius
        triangle[2] = projectedCenter + tangentDir0 * radius + tangentDir1 * radius

        callback.processTriangle(triangle, 0, 1)
    }

    override fun calculateLocalInertia(mass: Float, inertia: Vec3) {
        //moving concave objects not supported
        inertia put 0f
    }

    /** debugging   */
    override val name get()=  "STATICPLANE"
}