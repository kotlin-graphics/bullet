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
import bullet.linearMath.*

/** The TriangleMeshShape is an internal concave triangle mesh interface. Don't use this class directly, use
 *  BvhTriangleMeshShape instead.   */
abstract class TriangleMeshShape
/** TriangleMeshShape constructor has been disabled/protected, so that users will not mistakenly use this class.
 *  Don't use TriangleMeshShape but use BvhTriangleMeshShape instead!   */
protected constructor(val meshInterface: StridingMeshInterface) : ConcaveShape() {

    val localAabbMin = Vec3()
    val localAabbMax = Vec3()

    init {
        shapeType = BroadphaseNativeTypes.TRIANGLE_MESH_SHAPE_PROXYTYPE
        if (meshInterface.hasPremadeAabb()) meshInterface.getPremadeAabb(localAabbMin, localAabbMax)
        else recalcLocalAabb()
    }

    fun localGetSupportingVertex(vec: Vec3): Vec3 {
        val ident = Transform().apply { setIdentity() }
        val supportCallback = SupportVertexCallback(vec, ident)
        val aabbMax = Vec3(LARGE_FLOAT)
        processAllTriangles(supportCallback, -aabbMax, aabbMax)
        return supportCallback.supportVertexLocal  // supportVertex, TODO check if copy needed
    }

    fun localGetSupportingVertexWithoutMargin(vec: Vec3): Vec3 {
        assert(false)
        return localGetSupportingVertex(vec)
    }

    fun recalcLocalAabb() {
        for (i in 0..2) {
            val vec = Vec3().apply { set(i, 1f) }
            val tmp = localGetSupportingVertex(vec)
            localAabbMax[i] = tmp[i] + collisionMargin
            vec[i] = -1f
            tmp put localGetSupportingVertex(vec)
            localAabbMin[i] = tmp[i] - collisionMargin
        }
    }

    override fun getAabb(trans: Transform, aabbMin: Vec3, aabbMax: Vec3) {
        val localHalfExtents = 0.5f * (localAabbMax - localAabbMin)
        localHalfExtents += margin
        val localCenter = 0.5f * (localAabbMax + localAabbMin)

        val absB = trans.basis.absolute()

        val center = trans(localCenter)

        val extent = localHalfExtents dot3 absB
        aabbMin put center - extent // TODO order
        aabbMax put center + extent
    }

    override fun processAllTriangles(callback: TriangleCallback, aabbMin: Vec3, aabbMax: Vec3) {

        class FilteredCallback(val callback: TriangleCallback, val aabbMin: Vec3, val aabbMax: Vec3) : InternalTriangleIndexCallback {

            override fun internalProcessTriangleIndex(triangle: Array<Vec3>, partId: Int, triangleIndex: Int) {
                if (testTriangleAgainstAabb2(triangle, aabbMin, aabbMax))
                    callback.processTriangle(triangle, partId, triangleIndex) //check aabb in triangle-space, before doing this
            }
        }

        val filterCallback = FilteredCallback(callback, aabbMin, aabbMax)
        meshInterface.internalProcessAllTriangles(filterCallback, aabbMin, aabbMax)
    }

    override fun calculateLocalInertia(mass: Float, inertia: Vec3) {
        assert(false, { "moving concave objects not supported" })
        inertia put 0f
    }

    override var localScaling // TODO search for potential bug
        get() = meshInterface.scaling
        set(value) {
            meshInterface.scaling put value
            recalcLocalAabb()
        }

    /** debugging */
    override val name get() = "TRIANGLEMESH"
}

class SupportVertexCallback(supportVecWorld: Vec3, val worldTrans: Transform) : TriangleCallback {

    val supportVertexLocal = Vec3()
    var maxDot = -LARGE_FLOAT
    val supportVecLocal = supportVecWorld * worldTrans.basis

    override fun processTriangle(triangle: Array<Vec3>, partId: Int, triangleIndex: Int) {
        for (t in triangle) {
            val dot = supportVecLocal dot t
            if (dot > maxDot) {
                maxDot = dot
                supportVertexLocal put t
            }
        }
    }

    val supportVertexWorldSpace get() = worldTrans(supportVertexLocal)
}