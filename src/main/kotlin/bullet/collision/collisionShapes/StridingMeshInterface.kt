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
import bullet.linearMath.Vec3

/**	The StridingMeshInterface is the interface class for high performance generic access to triangle meshes, used in
 *  combination with BvhTriangleMeshShape and some other collision shapes.
 *  Using index striding of 3 * Int.BYTES it can use triangle arrays, using index striding of 1 * Int.BYTES it can
 *  handle triangle strips.
 *  It allows for sharing graphics and collision meshes. Also it provides locking/unlocking of graphics meshes that
 *  are in gpu memory.  */
abstract class StridingMeshInterface {

    val scaling = Vec3(1f)

    fun internalProcessAllTriangles(callback: InternalTriangleIndexCallback, aabbMin: Vec3, aabbMax: Vec3) {

        var numTotalPhysicsVerts = 0
        val graphicsSubparts = numSubParts
        val triangle = Array(3, { Vec3() })
        val meshScaling = scaling

        // if the number of parts is big, the performance might drop due to the innerloop switch on indexType
        for (part in 0 until graphicsSubparts) {
            val mesh = getLockedReadOnlyVertexIndexBase(part)
            val vertexBase = mesh.vertexBase
            val numVert = mesh.numVertices
            val type = mesh.vertexType
            val stride = mesh.vertexStride
            val indexBase = mesh.triangleIndexBase
            val indexStride = mesh.triangleIndexStride
            val numTriangles = mesh.numTriangles
            val gfxIndexType = mesh.indexType

            numTotalPhysicsVerts += numTriangles * 3 // upper bound

            when (type) {
                PHY_ScalarType.FLOAT -> {
                    val vertices = mesh.vertices as FloatArray
                    when (gfxIndexType) {
                        PHY_ScalarType.INTEGER -> {
                            val indices = mesh.indices as IntArray
                            for (gfxIndex in 0 until numTriangles) {
                                val indexOffset = indexBase + gfxIndex * indexStride
                                triangle[0].put(vertices, vertexBase + indices[indexOffset] * stride, meshScaling)
                                triangle[1].put(vertices, vertexBase + indices[indexOffset + 1] * stride, meshScaling)
                                triangle[2].put(vertices, vertexBase + indices[indexOffset + 2] * stride, meshScaling)
                                callback.internalProcessTriangleIndex(triangle, part, gfxIndex)
                            }
                        }
                        PHY_ScalarType.SHORT -> {
                            val indices = mesh.indices as ShortArray
                            for (gfxIndex in 0 until numTriangles) {
                                val indexOffset = indexBase + gfxIndex * indexStride
                                triangle[0].put(vertices, vertexBase + indices[indexOffset] * stride, meshScaling)
                                triangle[1].put(vertices, vertexBase + indices[indexOffset + 1] * stride, meshScaling)
                                triangle[2].put(vertices, vertexBase + indices[indexOffset + 2] * stride, meshScaling)
                                callback.internalProcessTriangleIndex(triangle, part, gfxIndex)
                            }
                        }
                        PHY_ScalarType.UCHAR -> {
                            val indices = mesh.indices as ByteArray
                            for (gfxIndex in 0 until numTriangles) {
                                val indexOffset = indexBase + gfxIndex * indexStride
                                triangle[0].put(vertices, vertexBase + indices[indexOffset] * stride, meshScaling)
                                triangle[1].put(vertices, vertexBase + indices[indexOffset + 1] * stride, meshScaling)
                                triangle[2].put(vertices, vertexBase + indices[indexOffset + 2] * stride, meshScaling)
                                callback.internalProcessTriangleIndex(triangle, part, gfxIndex)
                            }
                        }
                        else -> assert(gfxIndexType == PHY_ScalarType.INTEGER || gfxIndexType == PHY_ScalarType.SHORT)
                    }
                }
                PHY_ScalarType.DOUBLE -> {
                    val vertices = mesh.vertices as DoubleArray
                    when (gfxIndexType) {
                        PHY_ScalarType.INTEGER -> {
                            val indices = mesh.indices as IntArray
                            for (gfxIndex in 0 until numTriangles) {
                                val indexOffset = indexBase + gfxIndex * indexStride
                                triangle[0].put(vertices, vertexBase + indices[indexOffset] * stride, meshScaling)
                                triangle[1].put(vertices, vertexBase + indices[indexOffset + 1] * stride, meshScaling)
                                triangle[2].put(vertices, vertexBase + indices[indexOffset + 2] * stride, meshScaling)
                                callback.internalProcessTriangleIndex(triangle, part, gfxIndex)
                            }
                        }
                        PHY_ScalarType.SHORT -> {
                            val indices = mesh.indices as ShortArray
                            for (gfxIndex in 0 until numTriangles) {
                                val indexOffset = indexBase + gfxIndex * indexStride
                                triangle[0].put(vertices, vertexBase + indices[indexOffset] * stride, meshScaling)
                                triangle[1].put(vertices, vertexBase + indices[indexOffset + 1] * stride, meshScaling)
                                triangle[2].put(vertices, vertexBase + indices[indexOffset + 2] * stride, meshScaling)
                                callback.internalProcessTriangleIndex(triangle, part, gfxIndex)
                            }
                        }
                        PHY_ScalarType.UCHAR -> {
                            val indices = mesh.indices as ByteArray
                            for (gfxIndex in 0 until numTriangles) {
                                val indexOffset = indexBase + gfxIndex * indexStride
                                triangle[0].put(vertices, vertexBase + indices[indexOffset] * stride, meshScaling)
                                triangle[1].put(vertices, vertexBase + indices[indexOffset + 1] * stride, meshScaling)
                                triangle[2].put(vertices, vertexBase + indices[indexOffset + 2] * stride, meshScaling)
                                callback.internalProcessTriangleIndex(triangle, part, gfxIndex)
                            }
                        }
                        else -> assert(gfxIndexType == PHY_ScalarType.INTEGER || gfxIndexType == PHY_ScalarType.SHORT)
                    }
                }
                else -> assert (type == PHY_ScalarType.FLOAT || type == PHY_ScalarType.DOUBLE)
            }
            unLockReadOnlyVertexBase(part)
        }
    }

    // brute force method to calculate aabb
    fun calculateAabbBruteForce(aabbMin: Vec3, aabbMax: Vec3) {

        class AabbCalculationCallback : InternalTriangleIndexCallback {

            val aabbMin = Vec3(LARGE_FLOAT)
            val aabbMax = Vec3(-LARGE_FLOAT)

            override fun internalProcessTriangleIndex(triangle: Array<Vec3>, partId: Int, triangleIndex: Int) {
                aabbMin setMin triangle[0]
                aabbMax setMax triangle[0]
                aabbMin setMin triangle[1]
                aabbMax setMax triangle[1]
                aabbMin setMin triangle[2]
                aabbMax setMax triangle[2]
            }
        }

        //first calculate the total aabb for all triangles
        val aabbCallback = AabbCalculationCallback()
        aabbMin put -LARGE_FLOAT
        aabbMax put LARGE_FLOAT
        internalProcessAllTriangles(aabbCallback, aabbMin, aabbMax)

        aabbMin put aabbCallback.aabbMin
        aabbMax put aabbCallback.aabbMax
    }

    /** Get read and write access to a subpart of a triangle mesh
     *  this subpart has a continuous array of vertices and indices
     *  in this way the mesh can be handled as chunks of memory with striding very similar to OpenGL vertexArray support
     *  make a call to lockVertexBase when the read and write access is finished    */
    abstract fun getLockedVertexIndexBase(subPart: Int = 0): IndexedMesh

    abstract fun getLockedReadOnlyVertexIndexBase(subPart: Int = 0): IndexedMesh

    /** unLockVertexBase finishes the access to a subpart of the triangle mesh
     *  make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished */
    abstract fun unLockVertexBase(subPart: Int)

    abstract fun unLockReadOnlyVertexBase(subPart: Int)

    /** numSubParts returns the number of seperate subparts, each subpart has a continuous array of vertices and indices    */
    open val numSubParts get() =0

    abstract fun preallocateVertices(numVerts:Int)
    abstract fun preallocateIndices(numIndices:Int)

    open fun hasPremadeAabb() = false
    abstract fun setPremadeAabb(aabbMin:Vec3, aabbMax:Vec3 )
    abstract fun getPremadeAabb(aabbMin:Vec3, aabbMax :Vec3)
}