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
import bullet.collision.broadphaseCollision.BroadphaseNativeTypes
import bullet.collision.broadphaseCollision.NodeOverlapCallback
import bullet.i
import bullet.linearMath.Vec3

val DISABLE_BVH = false // TODO move to assimp
val DEBUG_TRIANGLE_MESH = false

/** The BvhTriangleMeshShape is a static-triangle mesh shape, it can only be used for fixed/non-moving objects.
 *  If you required moving concave triangle meshes, it is recommended to perform convex decomposition using HACD,
 *  see Bullet/Demos/ConvexDecompositionDemo.
 *  Alternatively, you can use GimpactMeshShape for moving concave triangle meshes.
 *  BvhTriangleMeshShape has several optimizations, such as bounding volume hierarchy and cache friendly traversal
 *  for PlayStation 3 Cell SPU.
 *  It is recommended to enable useQuantizedAabbCompression for better memory usage.
 *  It takes a triangle mesh as input, for example a TriangleMesh or TriangleIndexVertexArray. The BvhTriangleMeshShape
 *  class allows for triangle mesh deformations by a refit or partialRefit method.
 *  Instead of building the bounding volume hierarchy acceleration structure, it is also possible to serialize (save)
 *  and deserialize (load) the structure from disk.
 *  See Demos\ConcaveDemo\ConcavePhysicsDemo.cpp for an example.    */
class BvhTriangleMeshShape
/** swap first two arguments in order to have an unique common private constructor */
private constructor(val useQuantizedAabbCompression: Boolean, meshInterface: StridingMeshInterface, buildBvh: Boolean = true)
    : TriangleMeshShape(meshInterface) {

    var bvh: OptimizedBvh? = null
    var triangleInfoMap: TriangleInfoMap? = null
    var ownsBvh = false

    init {
        shapeType = BroadphaseNativeTypes.TRIANGLE_MESH_SHAPE_PROXYTYPE
    }

    constructor(meshInterface: StridingMeshInterface, useQuantizedAabbCompression: Boolean, buildBvh: Boolean = true) :
            this(useQuantizedAabbCompression, meshInterface, buildBvh) {
        if (!DISABLE_BVH && buildBvh) buildOptimizedBvh()
    }

    /** optionally pass in a larger bvh aabb, used for quantization. This allows for deformations within this aabb */
    constructor(meshInterface: StridingMeshInterface, useQuantizedAabbCompression: Boolean, bvhAabbMin: Vec3, bvhAabbMax: Vec3,
                buildBvh: Boolean = true) : this(useQuantizedAabbCompression, meshInterface, buildBvh) {
        if (!DISABLE_BVH && buildBvh) {
            bvh = OptimizedBvh().apply { build(meshInterface, useQuantizedAabbCompression, bvhAabbMin, bvhAabbMax) }
            ownsBvh = true
        }
    }

    fun performRaycast(callback: TriangleCallback, raySource: Vec3, rayTarget: Vec3) {
        class MyNodeOverlapCallback(val callback: TriangleCallback, val meshInterface: StridingMeshInterface) : NodeOverlapCallback {

            override fun processNode(subPart: Int, triangleIndex: Int) {

                val triangle = Array(3, { Vec3() })
                val (vertices, vertexBase, _, type, stride, indices, indexBase, indexStride, _, indicesType) = meshInterface.getLockedReadOnlyVertexIndexBase(subPart)

                val gfxBase = indexBase + triangleIndex * indexStride
                assert(indicesType == PHY_ScalarType.INTEGER || indicesType == PHY_ScalarType.SHORT)
                val meshScaling = meshInterface.scaling
                for (j in 2 downTo 0) {
                    val graphicsIndex = when (indicesType) {
                        PHY_ScalarType.SHORT -> (indices as ShortArray)[gfxBase + j].i
                        else -> (indices as IntArray)[gfxBase + j]
                    }
                    when (type) {
                        PHY_ScalarType.FLOAT -> triangle[j].put(vertices as FloatArray, vertexBase + graphicsIndex * stride, meshScaling)
                        else -> triangle[j].put(vertices as DoubleArray, vertexBase + graphicsIndex * stride, meshScaling)
                    }
                }
                /* Perform ray vs. triangle collision here */
                callback.processTriangle(triangle, subPart, triangleIndex)
                meshInterface.unLockReadOnlyVertexBase(subPart)
            }
        }

        val myNodeCallback = MyNodeOverlapCallback(callback, meshInterface)
        bvh!!.reportRayOverlappingNodex(myNodeCallback, raySource, rayTarget)
    }

    fun performConvexcast(callback: TriangleCallback, raySource: Vec3, rayTarget: Vec3, aabbMin: Vec3, aabbMax: Vec3) {

        class MyNodeOverlapCallback(val callback: TriangleCallback, val meshInterface: StridingMeshInterface) : NodeOverlapCallback {

            override fun processNode(subPart: Int, triangleIndex: Int) {

                val triangle = Array(3, { Vec3() })
                val (vertices, vertexBase, _, type, stride, indices, indexBase, indexStride, _, indicesType) = meshInterface.getLockedReadOnlyVertexIndexBase(subPart)

                val gfxBase = indexBase + triangleIndex * indexStride
                assert(indicesType == PHY_ScalarType.INTEGER || indicesType == PHY_ScalarType.SHORT)

                val meshScaling = meshInterface.scaling
                for (j in 2 downTo 0) {
                    val graphicsIndex = when (indicesType) {
                        PHY_ScalarType.SHORT -> (indices as ShortArray)[gfxBase + j].i
                        else -> (indices as IntArray)[gfxBase + j]
                    }
                    when (type) {
                        PHY_ScalarType.FLOAT -> triangle[j].put(vertices as FloatArray, vertexBase + graphicsIndex * stride, meshScaling)
                        else -> triangle[j].put(vertices as DoubleArray, vertexBase + graphicsIndex * stride, meshScaling)
                    }
                }
                /* Perform ray vs. triangle collision here */
                callback.processTriangle(triangle, subPart, triangleIndex)
                meshInterface.unLockReadOnlyVertexBase(subPart)
            }
        }

        val myNodeCallback = MyNodeOverlapCallback(callback, meshInterface)
        bvh!!.reportBoxCastOverlappingNodex(myNodeCallback, raySource, rayTarget, aabbMin, aabbMax)
    }

    /** perform bvh tree traversal and report overlapping triangles to 'callback'   */
    override fun processAllTriangles(callback: TriangleCallback, aabbMin: Vec3, aabbMax: Vec3) {

        if (DISABLE_BVH)
        //brute force traverse all triangles
            super.processAllTriangles(callback, aabbMin, aabbMax)
        else {
            //first get all the nodes
            class MyNodeOverlapCallback(val callback: TriangleCallback, val meshInterface: StridingMeshInterface) : NodeOverlapCallback {
                val triangle = Array(3, { Vec3() })
                var numOverlap = 0

                override fun processNode(subPart: Int, triangleIndex: Int) {
                    numOverlap++

                    val (vertices, vertexBase, numVerts, type, stride, indices, indexBase, indexStride, numFaces, indicesType) =
                            meshInterface.getLockedReadOnlyVertexIndexBase(subPart)

                    val gfxBase = indexBase + triangleIndex * indexStride
                    assert(indicesType == PHY_ScalarType.INTEGER || indicesType == PHY_ScalarType.SHORT || indicesType == PHY_ScalarType.UCHAR)

                    val meshScaling = meshInterface.scaling
                    for (j in 2 downTo 0) {
                        val graphicsIndex = when (indicesType) {
                            PHY_ScalarType.SHORT -> (indices as ShortArray)[gfxBase + j].i
                            PHY_ScalarType.INTEGER -> (indices as IntArray)[gfxBase + j]
                            else -> (indices as ByteArray)[gfxBase + j].i
                        }
                        if (DEBUG_TRIANGLE_MESH) print("$graphicsIndex ,")
                        val graphicsBase = vertexBase + graphicsIndex * stride
                        when (type) {
                            PHY_ScalarType.FLOAT -> triangle[j].put(vertices as FloatArray, graphicsBase, meshScaling)
                            else -> triangle[j].put(vertices as DoubleArray, graphicsBase, meshScaling)
                        }
                        if (DEBUG_TRIANGLE_MESH) println("triangle vertices:${triangle[j].x},${triangle[j].y},${triangle[j].z}")
                    }
                    callback.processTriangle(triangle, subPart, triangleIndex)
                    meshInterface.unLockReadOnlyVertexBase(subPart)
                }
            }

            val myNodeCallback = MyNodeOverlapCallback(callback, meshInterface)
            bvh!!.reportAabbOverlappingNodex(myNodeCallback, aabbMin, aabbMax)
        }
    }

    fun refitTree(aabbMin: Vec3, aabbMax: Vec3) = bvh!!.refit(meshInterface, aabbMin, aabbMax).also { recalcLocalAabb() }

    /** for a fast incremental refit of parts of the tree. Note: the entire AABB of the tree will become more
     *  conservative, it never shrinks */
    fun partialRefitTree(aabbMin: Vec3, aabbMax: Vec3) {
        bvh!!.refitPartial(meshInterface, aabbMin, aabbMax)
        localAabbMin setMin aabbMin
        localAabbMax setMax aabbMax
    }

    /** debugging */
    override val name get() = "BVHTRIANGLEMESH"

    override var localScaling: Vec3 // TODO search for potential bug
        get() = super.localScaling
        set(value) {
            if ((localScaling - value).length2() > Float.EPSILON) {
                super.localScaling = value
                buildOptimizedBvh()
            }
        }

    val optimizedBvh get() = bvh

    fun setOptimizedBvh(bvh: OptimizedBvh, scaling: Vec3 = Vec3(1f)) {
        assert(this.bvh == null && !ownsBvh)
        this.bvh = bvh
        ownsBvh = false
        // update the scaling without rebuilding the bvh
        if ((localScaling - scaling).length2() > Float.EPSILON) super.localScaling = scaling
    }

    fun buildOptimizedBvh() {
        /*  localAabbMin/localAabbMax is already re-calculated in TriangleMeshShape. We could just scale aabb, but this
            needs some more work         */
        bvh = OptimizedBvh().apply {
            build(meshInterface, useQuantizedAabbCompression, localAabbMin, localAabbMax) //rebuild the bvh...
        }
        ownsBvh = true
    }
}
