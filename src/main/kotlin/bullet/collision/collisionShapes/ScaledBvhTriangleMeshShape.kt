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

import bullet.collision.broadphaseCollision.BroadphaseNativeTypes
import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import bullet.linearMath.div
import bullet.linearMath.times


/** The ScaledBvhTriangleMeshShape allows to instance a scaled version of an existing BvhTriangleMeshShape.
 *  Note that each BvhTriangleMeshShape still can have its own local scaling, independent from this
 *  ScaledBvhTriangleMeshShape 'localScaling'   */
class ScaledBvhTriangleMeshShape(val bvhTriMeshShape: BvhTriangleMeshShape, val localScaling_: Vec3) : ConcaveShape() {

    init {
        shapeType = BroadphaseNativeTypes.SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE
    }

    val childShape get() = bvhTriMeshShape

    override fun getAabb(trans: Transform, aabbMin: Vec3, aabbMax: Vec3) {

        val localAabbMin = Vec3(bvhTriMeshShape.localAabbMin)
        val localAabbMax = Vec3(bvhTriMeshShape.localAabbMax)

        val tmpLocalAabbMin = localAabbMin * localScaling
        val tmpLocalAabbMax = localAabbMax * localScaling

        for (i in 0..2) {
            localAabbMin[i] = if (localScaling[i] >= 0) tmpLocalAabbMin[i] else tmpLocalAabbMax[i]
            localAabbMax[i] = if (localScaling[i] <= 0) tmpLocalAabbMin[i] else tmpLocalAabbMax[i]
        }

        val localHalfExtents = 0.5f * (localAabbMax - localAabbMin)
        val margin = bvhTriMeshShape.margin
        localHalfExtents += margin
        val localCenter = 0.5f * (localAabbMax + localAabbMin)

        val absB = trans.basis.absolute()

        val center = trans(localCenter)

        val extent = localHalfExtents dot3 absB
        aabbMin put center - extent
        aabbMax put center + extent
    }

    override fun calculateLocalInertia(mass: Float, inertia: Vec3) {
        // don't make this a movable object!
//        assert(false)
    }

    class ScaledTriangleCallback(val originalCallback: TriangleCallback, val localScaling: Vec3) : TriangleCallback {
        override fun processTriangle(triangle: Array<Vec3>, partId: Int, triangleIndex: Int) {
            val newTriangle = Array(3) { triangle[it] * localScaling }
            originalCallback.processTriangle(newTriangle, partId, triangleIndex)
        }
    }

    override fun processAllTriangles(callback: TriangleCallback, aabbMin: Vec3, aabbMax: Vec3) {

        val scaledCallback = ScaledTriangleCallback(callback, localScaling)

        val invLocalScaling = 1f / localScaling
        val scaledAabbMin = Vec3()
        val scaledAabbMax = Vec3()

        // support negative scaling
        for (i in 0..2) {
            scaledAabbMin[i] = invLocalScaling[i] * if (localScaling[i] >= 0) aabbMin[i] else aabbMax[i]
            scaledAabbMax[i] = invLocalScaling[i] * if (localScaling[i] <= 0) aabbMin[i] else aabbMax[i]
        }

        bvhTriMeshShape.processAllTriangles(scaledCallback, scaledAabbMin, scaledAabbMax)
    }

    override val name get() = "SCALEDBVHTRIANGLEMESH"
}