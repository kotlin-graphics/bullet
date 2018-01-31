/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//http://msdn.microsoft.com/library/default.asp?url=/library/en-us/vclang/html/vclrf__m128.asp

package bullet.collision.broadphaseCollision

import bullet.*
import bullet.linearMath.*
import kotlin.experimental.and
import kotlin.experimental.or

/** Note: currently we have 16 bytes per quantized node */
val MAX_SUBTREE_SIZE_IN_BYTES = 2048

/** 10 gives the potential for 1024 parts, with at most 2^21 (2097152) (minus one actually) triangles each
 *  (since the sign bit is reserved */
val MAX_NUM_PARTS_IN_BITS = 10

val DEBUG_CHECK_DEQUANTIZATION = false
val DEBUG_TREE_BUILDING = false
val VISUALLY_ANALYZE_BVH = false
val RAYAABB2 = true

/** QuantizedBvhNode is a compressed aabb node, 16 bytes.
 *  Node can be used for leafnode or internal node. Leafnodes can point to 32-bit triangle index (non-negative range).  */
class QuantizedBvhNode {
    //12 bytes
    val quantizedAabbMin = ShortArray(3)
    val quantizedAabbMax = ShortArray(3)
    //4 bytes
    var escapeIndexOrTriangleIndex = 0

    val isLeafNode get () = escapeIndexOrTriangleIndex >= 0 //skipindex is negative (internal node), triangleindex >=0 (leafnode)

    val escapeIndex: Int
        get() {
            assert(!isLeafNode)
            return -escapeIndexOrTriangleIndex
        }
    val triangleIndex: Int
        get() {
            assert(isLeafNode)
            val y = 0xffffffff.i shl (31 - MAX_NUM_PARTS_IN_BITS)
            // Get only the lower bits where the triangle index is stored
            return escapeIndexOrTriangleIndex and y.inv()
        }
    val partId: Int
        get() {
            assert(isLeafNode)
            // Get only the highest bits where the part index is stored
            return escapeIndexOrTriangleIndex ushr (31 - MAX_NUM_PARTS_IN_BITS)
        }

    companion object {
        val size = 6 * 2 + 4
    }
}

/** OptimizedBvhNode contains both internal and leaf node information.
 *  Total node size is 44 bytes / node. You can use the compressed version of 16 bytes. */
class OptimizedBvhNode {
    // 32 bytes
    val aabbMinOrg = Vec3()
    val aabbMaxOrg = Vec3()
    // 4
    var escapeIndex = 0
    // 8, for child nodes
    var subPart = 0
    var triangleIndex = 0
}

/** BvhSubtreeInfo provides info to gather a subtree of limited size */
class BvhSubtreeInfo {
    // 12 bytes
    val quantizedAabbMin = ShortArray(3)
    val quantizedAabbMax = ShortArray(3)
    // 4 bytes, points to the root of the subtree
    var rootNodeIndex = 0
    //4 bytes
    var subtreeSize = 0

    fun setAabbFromQuantizeNode(quantizedNode: QuantizedBvhNode) {
        quantizedAabbMin[0] = quantizedNode.quantizedAabbMin[0]
        quantizedAabbMin[1] = quantizedNode.quantizedAabbMin[1]
        quantizedAabbMin[2] = quantizedNode.quantizedAabbMin[2]
        quantizedAabbMax[0] = quantizedNode.quantizedAabbMax[0]
        quantizedAabbMax[1] = quantizedNode.quantizedAabbMax[1]
        quantizedAabbMax[2] = quantizedNode.quantizedAabbMax[2]
    }
}

interface NodeOverlapCallback {
    fun processNode(subPart: Int, triangleIndex: Int)
}

/** The QuantizedBvh class stores an AABB tree that can be quickly traversed on CPU and Cell SPU.
 *  It is used by the btBvhTriangleMeshShape as midphase.
 *  It is recommended to use quantization for better performance and lower memory requirements. */
open class QuantizedBvh {

    enum class TraversalMode { STACKLESS, STACKLESS_CACHE_FRIENDLY, RECURSIVE }

    val bvhAabbMin = Vec3(-Float.MAX_VALUE)
    val bvhAabbMax = Vec3(Float.MAX_VALUE)
    val bvhQuantization = Vec3()
    /** for serialization versioning. It could also be used to detect endianess.    */
    var bulletVersion = BULLET_VERSION

    var curNodeIndex = 0
    /** quantization data */
    var useQuantization = false

    var leafNodes = ArrayList<OptimizedBvhNode>()
    var contiguousNodes = ArrayList<OptimizedBvhNode>()
    val quantizedLeafNodes = ArrayList<QuantizedBvhNode>()
    val quantizedContiguousNodes = ArrayList<QuantizedBvhNode>()

    var traversalMode = TraversalMode.STACKLESS
    var subtreeHeaders = ArrayList<BvhSubtreeInfo>()

    /** This is only used for serialization so we don't have to add serialization directly to btAlignedObjectArray */
    var subtreeHeaderCount = 0

    /** two versions, one for quantized and normal nodes. This allows code-reuse while maintaining readability (no template/macro!)
     *  this might be refactored into a virtual, it is usually not calculated at run-time   */
    fun setInternalNodeAabbMin(nodeIndex: Int, aabbMin: Vec3) {
        if (useQuantization)
            quantize(quantizedContiguousNodes[nodeIndex].quantizedAabbMin, aabbMin, false)
        else
            contiguousNodes[nodeIndex].aabbMinOrg put aabbMin
    }

    fun setInternalNodeAabbMax(nodeIndex: Int, aabbMax: Vec3) = when {
        useQuantization -> quantize(quantizedContiguousNodes[nodeIndex].quantizedAabbMax, aabbMax, true)
        else -> contiguousNodes[nodeIndex].aabbMaxOrg put aabbMax
    }

    fun getAabbMin(nodeIndex: Int) = when {
        useQuantization -> unQuantize(quantizedLeafNodes[nodeIndex].quantizedAabbMin)
        else -> leafNodes[nodeIndex].aabbMinOrg //non-quantized
    }

    fun getAabbMax(nodeIndex: Int) = when {
        useQuantization -> unQuantize(quantizedLeafNodes[nodeIndex].quantizedAabbMax)
        else -> leafNodes[nodeIndex].aabbMaxOrg //non-quantized

    }

    fun setInternalNodeEscapeIndex(nodeIndex: Int, escapeIndex: Int) = when {
        useQuantization -> quantizedContiguousNodes[nodeIndex].escapeIndexOrTriangleIndex = -escapeIndex
        else -> contiguousNodes[nodeIndex].escapeIndex = escapeIndex
    }

    fun mergeInternalNodeAabb(nodeIndex: Int, newAabbMin: Vec3, newAabbMax: Vec3) {
        if (useQuantization) {
            val quantizedAabbMin = ShortArray(3)
            val quantizedAabbMax = ShortArray(3)
            quantize(quantizedAabbMin, newAabbMin, false)
            quantize(quantizedAabbMax, newAabbMax, true)
            for (i in 0..2) {
                if (quantizedContiguousNodes[nodeIndex].quantizedAabbMin[i] > quantizedAabbMin[i])
                    quantizedContiguousNodes[nodeIndex].quantizedAabbMin[i] = quantizedAabbMin[i]
                if (quantizedContiguousNodes[nodeIndex].quantizedAabbMax[i] < quantizedAabbMax[i])
                    quantizedContiguousNodes[nodeIndex].quantizedAabbMax[i] = quantizedAabbMax[i]
            }
        } else {
            //non-quantized
            contiguousNodes[nodeIndex].aabbMinOrg setMin newAabbMin
            contiguousNodes[nodeIndex].aabbMaxOrg setMax newAabbMax
        }
    }

    fun swapLeafNodes(firstIndex: Int, secondIndex: Int) {
        if (useQuantization) {
            val tmp = quantizedLeafNodes[firstIndex]
            quantizedLeafNodes[firstIndex] = quantizedLeafNodes[secondIndex]
            quantizedLeafNodes[secondIndex] = tmp
        } else {
            val tmp = leafNodes[firstIndex]
            leafNodes[firstIndex] = leafNodes[secondIndex]
            leafNodes[secondIndex] = tmp
        }
    }

    fun assignInternalNodeFromLeafNode(internalNode: Int, leafNodeIndex: Int) = when {
        useQuantization -> quantizedContiguousNodes[internalNode] = quantizedLeafNodes[leafNodeIndex]
        else -> contiguousNodes[internalNode] = leafNodes[leafNodeIndex]
    }

    var stackDepth = 0
    var maxStackDepth = 0

    fun buildTree(startIndex: Int, endIndex: Int) {

        if (DEBUG_TREE_BUILDING) {
            stackDepth++
            if (stackDepth > maxStackDepth) maxStackDepth = stackDepth
        }

        var splitAxis = 0
        var splitIndex = 0
        val numIndices = endIndex - startIndex
        val curIndex = curNodeIndex

        assert(numIndices > 0)

        if (numIndices == 1) {
            if (DEBUG_TREE_BUILDING) stackDepth--
            assignInternalNodeFromLeafNode(curNodeIndex, startIndex)
            curNodeIndex++
            return
        }
        //calculate Best Splitting Axis and where to split it. Sort the incoming 'leafNodes' array within range 'startIndex/endIndex'.
        splitAxis = calcSplittingAxis(startIndex, endIndex)

        splitIndex = sortAndCalcSplittingIndex(startIndex, endIndex, splitAxis)

        val internalNodeIndex = curNodeIndex

        /*  set the min aabb to 'inf' or a max value, and set the max aabb to a -inf/minimum value.
            the aabb will be expanded during buildTree/mergeInternalNodeAabb with actual node values         */
        setInternalNodeAabbMin(curNodeIndex, bvhAabbMax)//can't use btVector3(SIMD_INFINITY,SIMD_INFINITY,SIMD_INFINITY)) because of quantization
        setInternalNodeAabbMax(curNodeIndex, bvhAabbMin)//can't use btVector3(-SIMD_INFINITY,-SIMD_INFINITY,-SIMD_INFINITY)) because of quantization

        for (i in startIndex until endIndex)
            mergeInternalNodeAabb(curNodeIndex, getAabbMin(i), getAabbMax(i))

        curNodeIndex++
        //internalNode->m_escapeIndex;
        val leftChildNodexIndex = curNodeIndex
        buildTree(startIndex, splitIndex) //build left child tree

        val rightChildNodexIndex = curNodeIndex
        buildTree(splitIndex, endIndex) //build right child tree

        if (DEBUG_TREE_BUILDING) stackDepth--

        val escapeIndex = curNodeIndex - curIndex

        if (useQuantization) {
            // escapeIndex is the number of nodes of this subtree
            val sizeQuantizedNode = QuantizedBvhNode.size
            val treeSizeInBytes = escapeIndex * sizeQuantizedNode
            if (treeSizeInBytes > MAX_SUBTREE_SIZE_IN_BYTES)
                updateSubtreeHeaders(leftChildNodexIndex, rightChildNodexIndex)
        }
        setInternalNodeEscapeIndex(internalNodeIndex, escapeIndex)
    }

    fun calcSplittingAxis(startIndex: Int, endIndex: Int): Int {

        val means = Vec3()
        val variance = Vec3()
        val numIndices = endIndex - startIndex

        for (i in startIndex until endIndex)
            means += 0.5f * (getAabbMax(i) + getAabbMin(i)) // center
        means *= 1f / numIndices

        for (i in startIndex until endIndex) {
            val center = 0.5f * (getAabbMax(i) + getAabbMin(i))
            val diff2 = center - means
            variance += diff2 * diff2
        }
        variance *= 1f / (numIndices - 1)
        return variance.maxAxis()
    }

    fun sortAndCalcSplittingIndex(startIndex: Int, endIndex: Int, splitAxis: Int): Int {
        var splitIndex = startIndex
        val numIndices = endIndex - startIndex
        var splitValue = 0f

        val means = Vec3()
        for (i in startIndex until endIndex)
            means += 0.5f * (getAabbMax(i) + getAabbMin(i))
        means *= 1f / numIndices

        splitValue = means[splitAxis]

        //sort leafNodes so all values larger then splitValue comes first, and smaller values start from 'splitIndex'.
        for (i in startIndex until endIndex) {
            val center = 0.5f * (getAabbMax(i) + getAabbMin(i))
            if (center[splitAxis] > splitValue) {
                swapLeafNodes(i, splitIndex) //swap
                splitIndex++
            }
        }
        /*  if the splitIndex causes unbalanced trees, fix this by using the center in between startIndex and endIndex
            otherwise the tree-building might fail due to stack-overflows in certain cases.
            unbalanced1 is unsafe: it can cause stack overflows
            bool unbalanced1 = ((splitIndex==startIndex) || (splitIndex == (endIndex-1)));

            unbalanced2 should work too: always use center (perfect balanced trees)
            bool unbalanced2 = true;    */
        // this should be safe too:
        val rangeBalancedIndices = numIndices / 3
        val unbalanced = splitIndex <= (startIndex + rangeBalancedIndices) || splitIndex >= (endIndex - 1 - rangeBalancedIndices)

        if (unbalanced) splitIndex = startIndex + (numIndices ushr 1)

        val unbal = splitIndex == startIndex || splitIndex == endIndex
        assert(!unbal)

        return splitIndex
    }

    var maxIterations = 0

    fun walkStacklessTree(nodeCallback: NodeOverlapCallback, aabbMin: Vec3, aabbMax: Vec3) {
        assert(!useQuantization)

        var rootNode = contiguousNodes[0]
        var escapeIndex = 0
        var curIndex = 0
        var walkIterations = 0
        var isLeafNode = false
        //PCK: unsigned instead of bool
        var aabbOverlap = false

        while (curIndex < curNodeIndex) {
            //catch bugs in tree data
            assert(walkIterations < curNodeIndex)
            walkIterations++
            aabbOverlap = testAabbAgainstAabb2(aabbMin, aabbMax, rootNode.aabbMinOrg, rootNode.aabbMaxOrg)
            isLeafNode = rootNode.escapeIndex == -1
            //PCK: unsigned instead of bool
            if (isLeafNode && aabbOverlap)
                nodeCallback.processNode(rootNode.subPart, rootNode.triangleIndex)
            //PCK: unsigned instead of bool
            if (aabbOverlap || isLeafNode) {
                rootNode = contiguousNodes[contiguousNodes.indexOf(rootNode) + 1]
                curIndex++
            } else {
                escapeIndex = rootNode.escapeIndex
                rootNode = contiguousNodes[contiguousNodes.indexOf(rootNode) + escapeIndex]
                curIndex += escapeIndex
            }
        }
        if (maxIterations < walkIterations) maxIterations = walkIterations
    }

    var param = 0f

    fun walkStacklessQuantizedTreeAgainstRay(nodeCallback: NodeOverlapCallback, raySource: Vec3, rayTarget: Vec3,
                                             aabbMin: Vec3, aabbMax: Vec3, startNodeIndex: Int, endNodeIndex: Int) {
        assert(useQuantization)

        var curIndex = startNodeIndex
        var walkIterations = 0
        val subTreeSize = endNodeIndex - startNodeIndex

        var rootNode = quantizedContiguousNodes[startNodeIndex]
        var escapeIndex = 0

        var isLeafNode = false
        //PCK: unsigned instead of bool
        var boxBoxOverlap = false
        var rayBoxOverlap = false

        var lambdaMax = 1f

        val sign = IntArray(3)
        val rayDirection = Vec3()
        if (RAYAABB2) {
            rayDirection put (rayTarget - raySource).normalize()
            lambdaMax = rayDirection dot (rayTarget - raySource)
            ///what about division by zero? --> just set rayDirection[i] to 1.0
            rayDirection[0] = if (rayDirection[0] == 0f) LARGE_FLOAT else 1f / rayDirection[0]
            rayDirection[1] = if (rayDirection[1] == 0f) LARGE_FLOAT else 1f / rayDirection[1]
            rayDirection[2] = if (rayDirection[2] == 0f) LARGE_FLOAT else 1f / rayDirection[2]
            sign[0] = (rayDirection[0] < 0f).i
            sign[1] = (rayDirection[1] < 0f).i
            sign[2] = (rayDirection[2] < 0f).i
        }
        /* Quick pruning by quantized box */
        val rayAabbMin = raySource min rayTarget
        val rayAabbMax = raySource max rayTarget

        /* Add box cast extents to bounding box */
        rayAabbMin += aabbMin
        rayAabbMax += aabbMax

        val quantizedQueryAabbMin = ShortArray(3)
        val quantizedQueryAabbMax = ShortArray(3)
        quantizeWithClamp(quantizedQueryAabbMin, rayAabbMin, false)
        quantizeWithClamp(quantizedQueryAabbMax, rayAabbMax, true)

        while (curIndex < endNodeIndex) {

            if (VISUALLY_ANALYZE_BVH) {
                //some code snippet to debugDraw aabb, to visually analyze bvh structure
//                TODO static int drawPatch = 0
//                need some global access to a debugDrawer
//                extern btIDebugDraw * debugDrawerPtr
//                        if (curIndex == drawPatch) {
//                            btVector3 aabbMin, aabbMax
//                            aabbMin = unQuantize(rootNode->m_quantizedAabbMin)
//                            aabbMax = unQuantize(rootNode->m_quantizedAabbMax)
//                            btVector3 color (1, 0, 0)
//                            debugDrawerPtr->drawAabb(aabbMin, aabbMax, color)
//                        }
            }
            // catch bugs in tree data
            assert(walkIterations < subTreeSize)

            walkIterations++
            // PCK: unsigned instead of bool
            // only interested if this is closer than any previous hit
            param = 1f
            rayBoxOverlap = false
            boxBoxOverlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin, quantizedQueryAabbMax,
                    rootNode.quantizedAabbMin, rootNode.quantizedAabbMax)
            isLeafNode = rootNode.isLeafNode
            if (boxBoxOverlap) {
                val bounds = Array(2, { Vec3() })
                bounds[0] = unQuantize(rootNode.quantizedAabbMin)
                bounds[1] = unQuantize(rootNode.quantizedAabbMax)
                /* Add box cast extents */
                bounds[0] minusAssign aabbMax
                bounds[1] minusAssign aabbMin
//                btVector3 normal
                if (false) {
//                    val ra2 = rayAabb2(raySource, rayDirection, sign, bounds, param, 0.0, lambda_max)
//                    bool ra = btRayAabb (raySource, rayTarget, bounds[0], bounds[1], param, normal)
//                    if (ra2 != ra) {
//                        printf("functions don't match\n")
//                    }
                }
                rayBoxOverlap = if (RAYAABB2) {
                    /*  careful with this check: need to check division by zero (above) and fix the unQuantize method
                    thanks Joerg/hiker for the reproduction case!
                    http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=9&t=1858 */
                    BT_PROFILE("RayAabb2")
                    rayAabb2(raySource, rayDirection, sign, bounds, ::param, 0f, lambdaMax)
                } else true//btRayAabb(raySource, rayTarget, bounds[0], bounds[1], param, normal);
            }

            if (isLeafNode && rayBoxOverlap)
                nodeCallback.processNode(rootNode.partId, rootNode.triangleIndex)

            //PCK: unsigned instead of bool
            if (rayBoxOverlap || isLeafNode) {
                rootNode = quantizedContiguousNodes[quantizedContiguousNodes.indexOf(rootNode) + 1]
                curIndex++
            } else {
                escapeIndex = rootNode.escapeIndex
                rootNode = quantizedContiguousNodes[quantizedContiguousNodes.indexOf(rootNode) + escapeIndex]
                curIndex += escapeIndex
            }
        }
        if (maxIterations < walkIterations) maxIterations = walkIterations
    }

    fun walkStacklessQuantizedTree(nodeCallback: NodeOverlapCallback, quantizedQueryAabbMin: ShortArray,
                                   quantizedQueryAabbMax: ShortArray, startNodeIndex: Int, endNodeIndex: Int) {
        assert(useQuantization)

        var curIndex = startNodeIndex
        var walkIterations = 0
        val subTreeSize = endNodeIndex - startNodeIndex

        var rootNode = quantizedContiguousNodes[startNodeIndex]
        var escapeIndex = 0

        var isLeafNode = false
        //PCK: unsigned instead of bool
        var aabbOverlap = false

        while (curIndex < endNodeIndex) {
            if (VISUALLY_ANALYZE_BVH) {
                //some code snippet to debugDraw aabb, to visually analyze bvh structure
                //need some global access to a debugDrawer JVM TODO
//                extern btIDebugDraw * debugDrawerPtr
//                        if (curIndex == drawPatch) {
//                            btVector3 aabbMin, aabbMax
//                            aabbMin = unQuantize(rootNode->m_quantizedAabbMin)
//                            aabbMax = unQuantize(rootNode->m_quantizedAabbMax)
//                            btVector3 color (1, 0, 0)
//                            debugDrawerPtr->drawAabb(aabbMin, aabbMax, color)
//                        }
            }
            //catch bugs in tree data
            assert(walkIterations < subTreeSize)

            walkIterations++
            //PCK: unsigned instead of bool
            aabbOverlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin, quantizedQueryAabbMax, rootNode.quantizedAabbMin,
                    rootNode.quantizedAabbMax)
            isLeafNode = rootNode.isLeafNode

            if (isLeafNode && aabbOverlap)
                nodeCallback.processNode(rootNode.partId, rootNode.triangleIndex)
            //PCK: unsigned instead of bool
            if (aabbOverlap || isLeafNode) {
                rootNode = quantizedContiguousNodes[quantizedContiguousNodes.indexOf(rootNode) + 1]
                curIndex++
            } else {
                escapeIndex = rootNode.escapeIndex
                rootNode = quantizedContiguousNodes[quantizedContiguousNodes.indexOf(rootNode) + escapeIndex]
                curIndex += escapeIndex
            }
        }
        if (maxIterations < walkIterations) maxIterations = walkIterations
    }

    fun testQuantizedAabbAgainstQuantizedAabb(aabbMin1: ShortArray, aabbMax1: ShortArray, aabbMin2: ShortArray, aabbMax2: ShortArray) =
            (aabbMin1[0] > aabbMax2[0] || aabbMax1[0] < aabbMin2[0]) && (aabbMin1[2] > aabbMax2[2] || aabbMax1[2] < aabbMin2[2]) &&
                    (aabbMin1[1] > aabbMax2[1] || aabbMax1[1] < aabbMin2[1])

    fun walkStacklessTreeAgainstRay(nodeCallback: NodeOverlapCallback, raySource: Vec3, rayTarget: Vec3, aabbMin: Vec3, aabbMax: Vec3,
                                    startNodeIndex: Int, endNodeIndex: Int) {
        assert(!useQuantization)

        var rootNode = contiguousNodes[0]
        var escapeIndex = 0
        var curIndex = 0
        var walkIterations = 0
        var isLeafNode = false
        //PCK: unsigned instead of bool
        var aabbOverlap = false
        var rayBoxOverlap = false
        var lambdaMax = 1f

        /* Quick pruning by quantized box */
        val rayAabbMin = raySource min rayTarget
        val rayAabbMax = raySource max rayTarget

        /* Add box cast extents to bounding box */
        rayAabbMin += aabbMin
        rayAabbMax += aabbMax

        val rayDirectionInverse = Vec3()
        val sign = IntArray(3)
        if (RAYAABB2) {
            val rayDir = (rayTarget - raySource).normalize()
            lambdaMax = rayDir.dot(rayTarget - raySource)
            ///what about division by zero? --> just set rayDirection[i] to 1.0
            rayDirectionInverse.put(
                    if (rayDir[0] == 0f) LARGE_FLOAT else 1f / rayDir[0],
                    if (rayDir[1] == 0f) LARGE_FLOAT else 1f / rayDir[1],
                    if (rayDir[2] == 0f) LARGE_FLOAT else 1f / rayDir[2])
            sign[0] = (rayDirectionInverse[0] < 0f).i
            sign[1] = (rayDirectionInverse[1] < 0f).i
            sign[2] = (rayDirectionInverse[2] < 0f).i
        }

        val bounds = Array(2, { Vec3() })

        while (curIndex < curNodeIndex) {
            param = 1f
            //catch bugs in tree data
            assert(walkIterations < curNodeIndex)

            walkIterations++

            bounds[0] = rootNode.aabbMinOrg
            bounds[1] = rootNode.aabbMaxOrg
            /* Add box cast extents */
            bounds[0] minusAssign aabbMax
            bounds[1] minusAssign aabbMin

            aabbOverlap = testAabbAgainstAabb2(rayAabbMin, rayAabbMax, rootNode.aabbMinOrg, rootNode.aabbMaxOrg)
            //perhaps profile if it is worth doing the aabbOverlap test first

            if (RAYAABB2) {
                ///careful with this check: need to check division by zero (above) and fix the unQuantize method
                ///thanks Joerg/hiker for the reproduction case!
                ///http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=9&t=1858
                rayBoxOverlap = if (aabbOverlap)
                    rayAabb2(raySource, rayDirectionInverse, sign, bounds, ::param, 0f, lambdaMax)
                else false
            } else {
                val normal = Vec3()
                rayBoxOverlap = rayAabb(raySource, rayTarget, bounds[0], bounds[1], ::param, normal)
            }

            isLeafNode = rootNode.escapeIndex == -1

            //PCK: unsigned instead of bool
            if (isLeafNode && rayBoxOverlap)
                nodeCallback.processNode(rootNode.subPart, rootNode.triangleIndex)

            //PCK: unsigned instead of bool
            if (rayBoxOverlap || isLeafNode) {
                rootNode = contiguousNodes[contiguousNodes.indexOf(rootNode) + 1]
                curIndex++
            } else {
                escapeIndex = rootNode.escapeIndex
                rootNode = contiguousNodes[contiguousNodes.indexOf(rootNode) + escapeIndex]
                curIndex += escapeIndex
            }
        }
        if (maxIterations < walkIterations) maxIterations = walkIterations
    }

    /** tree traversal designed for small-memory processors like PS3 SPU */
    fun walkStacklessQuantizedTreeCacheFriendly(nodeCallback: NodeOverlapCallback, quantizedQueryAabbMin: ShortArray,
                                                quantizedQueryAabbMax: ShortArray) {
        assert(useQuantization)
        subtreeHeaders.forEach {
            //PCK: unsigned instead of bool
            if (testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin, quantizedQueryAabbMax, it.quantizedAabbMin,
                            it.quantizedAabbMax))  // overlap
                walkStacklessQuantizedTree(nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax, it.rootNodeIndex,
                        it.rootNodeIndex + it.subtreeSize)
        }
    }

    /** use the 16-byte stackless 'skipindex' node tree to do a recursive traversal */
    fun walkRecursiveQuantizedTreeAgainstQueryAabb(nodes: ArrayList<QuantizedBvhNode>, currentNode: Int,
                                                   nodeCallback: NodeOverlapCallback, quantizedQueryAabbMin: ShortArray,
                                                   quantizedQueryAabbMax: ShortArray) {
        assert(useQuantization)
        //PCK: unsigned instead of bool
        val aabbOverlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin, quantizedQueryAabbMax,
                nodes[currentNode].quantizedAabbMin, nodes[currentNode].quantizedAabbMax)
        val isLeafNode = nodes[currentNode].isLeafNode
        //PCK: unsigned instead of bool
        if (aabbOverlap)
            if (isLeafNode)
                nodeCallback.processNode(nodes[currentNode].partId, nodes[currentNode].triangleIndex)
            else {
                //process left and right children
                val leftChildNode = currentNode + 1
                walkRecursiveQuantizedTreeAgainstQueryAabb(nodes, leftChildNode, nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax)
                val rightChildNode = if (nodes[leftChildNode].isLeafNode) leftChildNode + 1 else leftChildNode + nodes[leftChildNode].escapeIndex
                walkRecursiveQuantizedTreeAgainstQueryAabb(nodes, rightChildNode, nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax)
            }
    }

    /** use the 16-byte stackless 'skipindex' node tree to do a recursive traversal */
//    void walkRecursiveQuantizedTreeAgainstQuantizedTree (const btQuantizedBvhNode * treeNodeA, const btQuantizedBvhNode* treeNodeB, btNodeOverlapCallback* nodeCallback)
//    const TODO cant find implementation on c++


    fun updateSubtreeHeaders(leftChildNodexIndex: Int, rightChildNodexIndex: Int) {
        assert(useQuantization)

        val leftChildNode = quantizedContiguousNodes[leftChildNodexIndex]
        val leftSubTreeSize = if (leftChildNode.isLeafNode) 1 else leftChildNode.escapeIndex
        val leftSubTreeSizeInBytes = leftSubTreeSize * QuantizedBvhNode.size

        val rightChildNode = quantizedContiguousNodes[rightChildNodexIndex]
        val rightSubTreeSize = if (rightChildNode.isLeafNode) 1 else rightChildNode.escapeIndex
        val rightSubTreeSizeInBytes = rightSubTreeSize * QuantizedBvhNode.size

        if (leftSubTreeSizeInBytes <= MAX_SUBTREE_SIZE_IN_BYTES)
            subtreeHeaders.add(BvhSubtreeInfo().apply {
                setAabbFromQuantizeNode(leftChildNode)
                rootNodeIndex = leftChildNodexIndex
                subtreeSize = leftSubTreeSize
            })

        if (rightSubTreeSizeInBytes <= MAX_SUBTREE_SIZE_IN_BYTES)
            subtreeHeaders.add(BvhSubtreeInfo().apply {
                setAabbFromQuantizeNode(rightChildNode)
                rootNodeIndex = rightChildNodexIndex
                subtreeSize = rightSubTreeSize
            })
        //PCK: update the copy of the size
        subtreeHeaderCount = subtreeHeaders.size
    }


    ///***************************************** expert/internal use only *************************
    fun setQuantizationValues(bvhAabbMin: Vec3, bvhAabbMax: Vec3, quantizationMargin: Float = 1f) {
        //enlarge the AABB to avoid division by zero when initializing the quantization values
        val clampValue = Vec3(quantizationMargin)
        this.bvhAabbMin put bvhAabbMin - clampValue
        this.bvhAabbMax put bvhAabbMax + clampValue
        val aabbSize = this.bvhAabbMax - this.bvhAabbMin
        bvhQuantization put 65533f / aabbSize

        useQuantization = true

        val vecIn = ShortArray(3)
        quantize(vecIn, this.bvhAabbMin, false)
        val v = unQuantize(vecIn)
        this.bvhAabbMin setMin (v - clampValue)
        aabbSize put this.bvhAabbMax - this.bvhAabbMin
        bvhQuantization put 65533f / aabbSize

        quantize(vecIn, bvhAabbMax, true)
        v put unQuantize(vecIn)
        this.bvhAabbMax setMax (v + clampValue)

        aabbSize put this.bvhAabbMax - this.bvhAabbMin
        bvhQuantization put 65533f / aabbSize
    }

    val leafNodeArray get() = quantizedLeafNodes
    /** buildInternal is expert use only: assumes that setQuantizationValues and LeafNodeArray are initialized  */
    fun buildInternal() {
        // assumes that caller filled in the m_quantizedLeafNodes
        useQuantization = true
        var numLeafNodes = 0

        if (useQuantization) {
            numLeafNodes = quantizedLeafNodes.size //now we have an array of leafnodes in leafNodes
            quantizedContiguousNodes resize (2 * numLeafNodes)
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
///***************************************** expert/internal use only *************************

    fun reportAabbOverlappingNodex(nodeCallback: NodeOverlapCallback, aabbMin: Vec3, aabbMax: Vec3) {
        //either choose recursive traversal (walkTree) or stackless (walkStacklessTree)
        if (useQuantization) {
            ///quantize query AABB
            val quantizedQueryAabbMin = ShortArray(3)
            val quantizedQueryAabbMax = ShortArray(3)
            quantizeWithClamp(quantizedQueryAabbMin, aabbMin, false)
            quantizeWithClamp(quantizedQueryAabbMax, aabbMax, true)

            when (traversalMode) {
                TraversalMode.STACKLESS -> walkStacklessQuantizedTree(nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax, 0, curNodeIndex)
                TraversalMode.STACKLESS_CACHE_FRIENDLY -> walkStacklessQuantizedTreeCacheFriendly(nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax)
                TraversalMode.RECURSIVE -> {
                    val rootNode = 0
                    walkRecursiveQuantizedTreeAgainstQueryAabb(quantizedContiguousNodes, rootNode, nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax)
                }
            }
        } else walkStacklessTree(nodeCallback, aabbMin, aabbMax)
    }

    fun reportRayOverlappingNodex(nodeCallback: NodeOverlapCallback, raySource: Vec3, rayTarget: Vec3) =
            reportBoxCastOverlappingNodex(nodeCallback, raySource, rayTarget, Vec3(), Vec3())

    fun reportBoxCastOverlappingNodex(nodeCallback: NodeOverlapCallback, raySource: Vec3, rayTarget: Vec3, aabbMin: Vec3, aabbMax: Vec3) {
        // always use stackless
        if (useQuantization)
            walkStacklessQuantizedTreeAgainstRay(nodeCallback, raySource, rayTarget, aabbMin, aabbMax, 0, curNodeIndex)
        else
            walkStacklessTreeAgainstRay(nodeCallback, raySource, rayTarget, aabbMin, aabbMax, 0, curNodeIndex)
    }

    fun quantize(out: ShortArray, point: Vec3, isMax: Boolean) {

        assert(useQuantization)
        assert(point.x <= bvhAabbMax.x && point.y <= bvhAabbMax.y && point.z <= bvhAabbMax.z)
        assert(point.x >= bvhAabbMin.x && point.y >= bvhAabbMin.y && point.z >= bvhAabbMin.z)

        val v = (point - bvhAabbMin) * bvhQuantization
        /** Make sure rounding is done in a way that unQuantize(quantizeWithClamp(...)) is conservative
         *  end-points always set the first bit, so that they are sorted properly (so that neighbouring AABBs overlap properly)
         *  @todo: double-check this */
        if (isMax) {
            out[0] = (v.x + 1f).s or 1
            out[1] = (v.y + 1f).s or 1
            out[2] = (v.z + 1f).s or 1
        } else {
            out[0] = v.x.s and 0xfffe.s
            out[1] = v.y.s and 0xfffe.s
            out[2] = v.z.s and 0xfffe.s
        }

        if (DEBUG_CHECK_DEQUANTIZATION) with(unQuantize(out)) {
            if (isMax) {
                if (x < point.x) println("unconservative X, diffX = ${x - point.x}, oldX=$x,newX=${point.x}")
                if (y < point.y) println("unconservative Y, diffY = ${y - point.y}, oldY=$y,newY=${point.y}")
                if (z < point.z) println("unconservative Z, diffZ = ${z - point.z}, oldZ=$z,newZ=${point.z}")
            } else {
                if (x > point.x) println("unconservative X, diffX = ${x - point.x}, oldX=$x,newX=${point.x}")
                if (y > point.y) println("unconservative Y, diffY = ${y - point.y}, oldY=$y,newY=${point.y}")
                if (z > point.z) println("unconservative Z, diffZ = ${z - point.z}, oldZ=$z,newZ=${point.z}")
            }
        }
    }

    fun quantizeWithClamp(out: ShortArray, point2: Vec3, isMax: Boolean) {
        assert(useQuantization)
        val clampedPoint = Vec3(point2).apply { setMax(bvhAabbMin); setMin(bvhAabbMax) }
        quantize(out, clampedPoint, isMax)
    }

    fun unQuantize(vecIn: ShortArray) = Vec3(
            vecIn[0].f / bvhQuantization.x,
            vecIn[1].f / bvhQuantization.y,
            vecIn[2].f / bvhQuantization.z).apply { plusAssign(bvhAabbMin) }


    val quantizedNodeArray get() = quantizedContiguousNodes
    val subtreeInfoArray get() = subtreeHeaders
    val isQuantized get() = useQuantization

    companion object {
        var drawPatch = 0
    }
}