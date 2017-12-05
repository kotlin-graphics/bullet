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

///Contains contributions from Disney Studio's

package bullet.collision.collisionShapes

import bullet.collision.broadphaseCollision.*
import bullet.i
import bullet.linearMath.LARGE_FLOAT
import bullet.linearMath.Vec3
import bullet.resize

/** The OptimizedBvh extends the QuantizedBvh to create AABB tree for triangle meshes, through the StridingMeshInterface. */
class OptimizedBvh : QuantizedBvh() {

    fun build(triangles: StridingMeshInterface, useQuantizedAabbCompression: Boolean, bvhAabbMin: Vec3, bvhAabbMax: Vec3) {

        useQuantization = useQuantizedAabbCompression

        class NodeTriangleCallback(val triangleNodes: ArrayList<OptimizedBvhNode>) : InternalTriangleIndexCallback {

            operator fun invoke(other: NodeTriangleCallback): NodeTriangleCallback {
                triangleNodes.clear()
                triangleNodes.addAll(other.triangleNodes)
                return this
            }

            override fun internalProcessTriangleIndex(triangle: Array<Vec3>, partId: Int, triangleIndex: Int) {

                val node = OptimizedBvhNode()
                val aabbMin = Vec3(LARGE_FLOAT)
                val aabbMax = Vec3(-LARGE_FLOAT)
                aabbMin setMin triangle[0]
                aabbMax setMax triangle[0]
                aabbMin setMin triangle[1]
                aabbMax setMax triangle[1]
                aabbMin setMin triangle[2]
                aabbMax setMax triangle[2]

                //with quantization?
                node.aabbMinOrg put aabbMin
                node.aabbMaxOrg put aabbMax

                node.escapeIndex = -1

                //for child nodes
                node.subPart = partId
                node.triangleIndex = triangleIndex
                triangleNodes.add(node)
            }
        }

        class QuantizedNodeTriangleCallback(val triangleNodes: ArrayList<QuantizedBvhNode>,
                                            /** for quantization */
                                            var optimizedTree: QuantizedBvh) : InternalTriangleIndexCallback {

            operator fun invoke(other: QuantizedNodeTriangleCallback): QuantizedNodeTriangleCallback {
                triangleNodes.clear()
                triangleNodes.addAll(other.triangleNodes)
                optimizedTree = other.optimizedTree
                return this
            }

            override fun internalProcessTriangleIndex(triangle: Array<Vec3>, partId: Int, triangleIndex: Int) {

                // The partId and triangle index must fit in the same (positive) integer
                assert(partId < (1 shl MAX_NUM_PARTS_IN_BITS) && triangleIndex < (1 shl (31 - MAX_NUM_PARTS_IN_BITS)))
                // negative indices are reserved for escapeIndex
                assert(triangleIndex >= 0)

                val node = QuantizedBvhNode()
                val aabbMin = Vec3(LARGE_FLOAT)
                val aabbMax = Vec3(-LARGE_FLOAT)
                aabbMin setMin triangle[0]
                aabbMax setMax triangle[0]
                aabbMin setMin triangle[1]
                aabbMax setMax triangle[1]
                aabbMin setMin triangle[2]
                aabbMax setMax triangle[2]

                //PCK: add these checks for zero dimensions of aabb
                val MIN_AABB_DIMENSION = 0.002f
                val MIN_AABB_HALF_DIMENSION = 0.001f
                if (aabbMax.x - aabbMin.x < MIN_AABB_DIMENSION) {
                    aabbMax.x = aabbMax.x + MIN_AABB_HALF_DIMENSION
                    aabbMin.x = aabbMin.x - MIN_AABB_HALF_DIMENSION
                }
                if (aabbMax.y - aabbMin.y < MIN_AABB_DIMENSION) {
                    aabbMax.y = aabbMax.y + MIN_AABB_HALF_DIMENSION
                    aabbMin.y = aabbMin.y - MIN_AABB_HALF_DIMENSION
                }
                if (aabbMax.z - aabbMin.z < MIN_AABB_DIMENSION) {
                    aabbMax.z = aabbMax.z + MIN_AABB_HALF_DIMENSION
                    aabbMin.z = aabbMin.z - MIN_AABB_HALF_DIMENSION
                }
                optimizedTree.quantize(node.quantizedAabbMin, aabbMin, false)
                optimizedTree.quantize(node.quantizedAabbMax, aabbMax, true)

                node.escapeIndexOrTriangleIndex = (partId shl (31 - MAX_NUM_PARTS_IN_BITS)) or triangleIndex

                triangleNodes.add(node)
            }
        }

        var numLeafNodes = 0

        if (useQuantization) {
            //initialize quantization values
            setQuantizationValues(bvhAabbMin, bvhAabbMax)
            val callback = QuantizedNodeTriangleCallback(quantizedLeafNodes, this)
            triangles.internalProcessAllTriangles(callback, bvhAabbMin, bvhAabbMax)
            //now we have an array of leafnodes in m_leafNodes
            numLeafNodes = quantizedLeafNodes.size
            quantizedContiguousNodes resize (2 * numLeafNodes)
        } else {
            val callback = NodeTriangleCallback(leafNodes)
            val aabbMin = Vec3(-LARGE_FLOAT)
            val aabbMax = Vec3(LARGE_FLOAT)
            triangles.internalProcessAllTriangles(callback, aabbMin, aabbMax)
            //now we have an array of leafnodes in m_leafNodes
            numLeafNodes = leafNodes.size
            contiguousNodes resize (2 * numLeafNodes)
        }
        curNodeIndex = 0
        buildTree(0, numLeafNodes)
        // if the entire tree is small then subtree size, we need to create a header info for the tree
        if (useQuantization && subtreeHeaders.isEmpty())
            subtreeHeaders.add(BvhSubtreeInfo().apply {
                setAabbFromQuantizeNode(quantizedContiguousNodes[0])
                rootNodeIndex = 0
                subtreeSize = if (quantizedContiguousNodes[0].isLeafNode) 1 else quantizedContiguousNodes[0].escapeIndex
            })
        //PCK: update the copy of the size
        subtreeHeaderCount = subtreeHeaders.size
        //PCK: clear m_quantizedLeafNodes and m_leafNodes, they are temporary
        quantizedLeafNodes.clear()
        leafNodes.clear()
    }

    fun refit(meshInterface: StridingMeshInterface, aabbMin: Vec3, aabbMax: Vec3) {
        if (useQuantization) {
            setQuantizationValues(aabbMin, aabbMax)
            updateBvhNodes(meshInterface, 0, curNodeIndex, 0)
            // now update all subtree headers
            subtreeHeaders.forEach { it.setAabbFromQuantizeNode(quantizedContiguousNodes[it.rootNodeIndex]) }
        }
    }

    fun refitPartial(meshInterface: StridingMeshInterface, aabbMin: Vec3, aabbMax: Vec3) {
        // incrementally initialize quantization values
        assert(useQuantization)
        assert(aabbMin.x > bvhAabbMin.x && aabbMin.y > bvhAabbMin.y && aabbMin.z > bvhAabbMin.z)
        assert(aabbMax.x < bvhAabbMax.x && aabbMax.y < bvhAabbMax.y && aabbMax.z < bvhAabbMax.z)
        // we should update all quantization values, using updateBvhNodes(meshInterface);
        // but we only update chunks that overlap the given aabb
        val quantizedQueryAabbMin = ShortArray(3)
        val quantizedQueryAabbMax = ShortArray(3)

        quantize(quantizedQueryAabbMin, aabbMin, false)
        quantize(quantizedQueryAabbMax, aabbMax, true)

        for (i in 0 until subtreeHeaders.size) {
            val subtree = subtreeHeaders[i]
            //PCK: unsigned instead of bool
            val overlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin, quantizedQueryAabbMax,
                    subtree.quantizedAabbMin, subtree.quantizedAabbMax)
            if (overlap) {
                updateBvhNodes(meshInterface, subtree.rootNodeIndex, subtree.rootNodeIndex + subtree.subtreeSize, i)
                subtree.setAabbFromQuantizeNode(quantizedContiguousNodes[subtree.rootNodeIndex])
            }
        }
    }

    fun updateBvhNodes(meshInterface: StridingMeshInterface, firstNode: Int, endNode: Int, index: Int) {

        assert(useQuantization)

        var curNodeSubPart = -1
        //get access info to trianglemesh data
        lateinit var vertices: Any
        var vertexBase = 0
        var numVerts = 0
        var type = PHY_ScalarType.INTEGER
        var stride = 0
        lateinit var indices: Any
        var indexBase = 0
        var indexStride = 0
        var numFaces = 0
        var indicesType = PHY_ScalarType.INTEGER

        val triangleVerts = Array(3, { Vec3() })
        val aabbMin = Vec3()
        val aabbMax = Vec3()
        val meshScaling = meshInterface.scaling

        var i = endNode - 1
        while (i >= firstNode) {

            val curNode = quantizedContiguousNodes[i]
            if (curNode.isLeafNode) {
                //recalc aabb from triangle data
                val nodeSubPart = curNode.partId
                val nodeTriangleIndex = curNode.triangleIndex
                if (nodeSubPart != curNodeSubPart) {
                    if (curNodeSubPart >= 0)
                        meshInterface.unLockReadOnlyVertexBase(curNodeSubPart)
                    meshInterface.getLockedReadOnlyVertexIndexBase(nodeSubPart).also { info ->
                        vertices = info.vertices!!
                        vertexBase = info.vertexBase
                        numVerts = info.numVertices
                        type = info.vertexType
                        stride = info.vertexStride
                        indices = info.indices!!
                        indexBase = info.triangleIndexBase
                        indexStride = info.triangleIndexStride
                        numFaces = info.numTriangles
                        indicesType = info.indexType
                    }
                    curNodeSubPart = nodeSubPart
                    assert(indicesType == PHY_ScalarType.INTEGER || indicesType == PHY_ScalarType.SHORT)
                }
                val gfxBase = indexBase + nodeTriangleIndex * indexStride
                for (j in 2 downTo 0) {
                    val graphicsIndex = when (indicesType) {
                        PHY_ScalarType.SHORT -> (indices as ShortArray)[gfxBase + j].i
                        else -> (indices as IntArray)[gfxBase + j]
                    }
                    val graphicsBase = vertexBase + graphicsIndex * stride
                    when (type) {
                        PHY_ScalarType.FLOAT -> triangleVerts[j].put(vertices as FloatArray, graphicsBase, meshScaling)
                        else -> triangleVerts[j].put(vertices as DoubleArray, graphicsBase, meshScaling)
                    }
                }
                aabbMin put LARGE_FLOAT
                aabbMax put -LARGE_FLOAT
                aabbMin setMin triangleVerts[0]
                aabbMax setMax triangleVerts[0]
                aabbMin setMin triangleVerts[1]
                aabbMax setMax triangleVerts[1]
                aabbMin setMin triangleVerts[2]
                aabbMax setMax triangleVerts[2]

                quantize(curNode.quantizedAabbMin, aabbMin, false)
                quantize(curNode.quantizedAabbMax, aabbMax, true)

            } else {
                //combine aabb from both children
                val leftChildNode = quantizedContiguousNodes[i + 1]
                val rightChildNode = if (leftChildNode.isLeafNode) quantizedContiguousNodes[i + 2]
                else quantizedContiguousNodes[i + 1 + leftChildNode.escapeIndex]

                for (j in 0..2) {
                    curNode.quantizedAabbMin[j] = leftChildNode.quantizedAabbMin[i]
                    if (curNode.quantizedAabbMin[j] > rightChildNode.quantizedAabbMin[i])
                    curNode.quantizedAabbMin[j] = rightChildNode.quantizedAabbMin[i]

                    curNode.quantizedAabbMax[j] = leftChildNode.quantizedAabbMax[i]
                    if (curNode.quantizedAabbMax[j] < rightChildNode.quantizedAabbMax[i])
                    curNode.quantizedAabbMax[j] = rightChildNode.quantizedAabbMax[i]
                }
            }
            i--
        }
        if (curNodeSubPart >= 0) meshInterface.unLockReadOnlyVertexBase(curNodeSubPart)
    }
}
