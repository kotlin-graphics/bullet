/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//btDbvt implementation by Nathanael Presson

package bullet.collision.broadphaseCollision

import bullet.*
import bullet.linearMath.LARGE_FLOAT
import bullet.linearMath.Vec3
import bullet.linearMath.rayAabb
import bullet.linearMath.rayAabb2
import kotlin.math.abs
import kotlin.math.max
import kotlin.reflect.KMutableProperty0

/* btDbvtAabbMm			*/
class DbvtAabbMm(val min: Vec3 = Vec3(), val max: Vec3 = Vec3()) {

    val center get() = (min + max) / 2f
    val lengths get() = max - min
    val extents get() = (max - min) / 2f

    companion object {
        fun fromCE(c: Vec3, e: Vec3) = DbvtAabbMm(c - e, c + e)
        fun fromCR(c: Vec3, r: Float) = fromCE(c, Vec3(r))
        fun fromMM(mi: Vec3, mx: Vec3) = DbvtAabbMm(Vec3(mi), Vec3(mx))
        fun fromPoints(pts: Array<Vec3>) = DbvtAabbMm().apply { pts.forEach { min.setMin(it); max.setMax(it); } }
    }

    infix fun put(other: DbvtAabbMm) {
        min put other.min
        max put other.max
        center put other.center
        lengths put other.lengths
        extents put other.extents
    }

    infix fun expand(e: Float) {
        min -= e
        max += e
    }

//    infix fun expand(e: Vec3) {
//        min -= e
//        max += e
//    }

    infix fun signedExpand(e: Vec3) {
        if (e.x > 0) max.x = max.x + e[0] else min.x = min.x + e[0]
        if (e.y > 0) max.y = max.y + e[1] else min.y = min.y + e[1]
        if (e.z > 0) max.z = max.z + e[2] else min.z = min.z + e[2]
    }

    infix fun contain(a: DbvtAabbMm) = min.x <= a.min.x && min.y <= a.min.y && min.z <= a.min.z &&
            max.x >= a.max.x && max.y >= a.max.y && max.z >= a.max.z

    fun classify(n: Vec3, o: Float, s: Int): Int {
        val pi: Vec3
        val px: Vec3
        when (s) {
            0 + 0 + 0 -> {
                px = Vec3(min.x, min.y, min.z)
                pi = Vec3(max.x, max.y, max.z)
            }
            1 + 0 + 0 -> {
                px = Vec3(max.x, min.y, min.z)
                pi = Vec3(min.x, max.y, max.z)
            }
            0 + 2 + 0 -> {
                px = Vec3(min.x, max.y, min.z)
                pi = Vec3(max.x, min.y, max.z)
            }
            1 + 2 + 0 -> {
                px = Vec3(max.x, max.y, min.z)
                pi = Vec3(min.x, min.y, max.z)
            }
            0 + 0 + 4 -> {
                px = Vec3(min.x, min.y, max.z)
                pi = Vec3(max.x, max.y, min.z)
            }
            1 + 0 + 4 -> {
                px = Vec3(max.x, min.y, max.z)
                pi = Vec3(min.x, max.y, min.z)
            }
            0 + 2 + 4 -> {
                px = Vec3(min.x, max.y, max.z)
                pi = Vec3(max.x, min.y, min.z)
            }
            1 + 2 + 4 -> {
                px = Vec3(max.x, max.y, max.z)
                pi = Vec3(min.x, min.y, min.z)
            }
            else -> throw Error()
        }
        return when {
            (n dot px) + o < 0 -> -1
            (n dot pi) + o >= 0 -> +1
            else -> 0
        }
    }

    fun projectMinimum(v: Vec3, signs: Int): Float {
        val b = arrayOf(max, min)
        val p = Vec3(b[(signs ushr 0) and 1].x, b[(signs ushr 1) and 1].y, b[(signs ushr 2) and 1].z)
        return p dot v
    }

    infix fun intersect(b: DbvtAabbMm) = min.x <= b.max.x && max.x >= b.min.x && min.y <= b.max.y &&
            max.y >= b.min.y && min.z <= b.max.z && max.z >= b.min.z

    infix fun intersect(b: Vec3) = b.x >= min.x && b.y >= min.y && b.z >= min.z &&
            b.x <= max.x && b.y <= max.y && b.z <= max.z

    infix fun proximity(b: DbvtAabbMm): Float {
        val d = min + max - (b.min + b.max)
        return abs(d.x) + abs(d.y) + abs(d.z)
    }

    fun select(a: DbvtAabbMm, b: DbvtAabbMm) = if (proximity(a) < proximity(b)) 0 else 1
    fun merge(b: DbvtAabbMm, r: DbvtAabbMm) {
        for (i in 0..2) {
            r.min[i] = if (min[i] < b.min[i]) min[i] else b.min[i]
            r.max[i] = if (max[i] > b.max[i]) max[i] else b.max[i]
        }
    }

    override fun equals(other: Any?) = other is DbvtAabbMm && min == other.min && max == other.max
    override fun hashCode() = 31 * min.hashCode() + max.hashCode()

    fun addSpan(d: Vec3, s: FloatArray) {
        for (i in 0..2) {
            if (d[i] < 0) {
                s[0] += max[i] * d[i]
                s[1] += min[i] * d[i]
            } else {
                s[0] += min[i] * d[i]
                s[1] += max[i] * d[i]
            }
        }
    }

    /** volume + edge lengths */
    val size get() = with(lengths) { x * y * z + x + y + z }

    infix fun merge(b: DbvtVolume) = DbvtVolume().also { merge(b, it) }
}

// Types
typealias DbvtVolume = DbvtAabbMm

class DbvtNode {

    var volume = DbvtVolume()
    var parent: DbvtNode? = null

    inner class UnionArray {
        operator fun get(index: Int) = union[index] as? DbvtNode
        operator fun set(index: Int, node: DbvtNode?) = union.set(index, node)
    }

    private val union = Array<Any?>(2) { null }

    val childs = UnionArray()
    var data
        get() = union[0]
        set(value) = union.set(0, value)
    var dataAsInt
        get() = union[0] as Int
        set(value) = union.set(0, value)


    val isLeaf get() = childs[1] == null
    val isInternal get() = !isLeaf

    val indexOf get() = (parent!!.childs[1] === this).i
    /** JVM Specific, maxDepth modified in return!  */
    fun getMaxDepth(depth: Int, maxDepth: Int): Int {
        var res = maxDepth
        if (isInternal) {
            res = childs[0]!!.getMaxDepth(depth + 1, res)
            res = childs[1]!!.getMaxDepth(depth + 1, res)
        } else res = max(res, depth)
        return res
    }
}

/** The Dbvt class implements a fast dynamic bounding volume tree based on axis aligned bounding boxes (aabb tree).
 *  This Dbvt is used for soft body collision detection and for the DbvtBroadphase. It has a fast insert, remove and
 *  update of nodes.
 *  Unlike the QuantizedBvh, nodes can be dynamically moved around, which allows for change in topology
 *  of the underlying data structure.   */
class Dbvt {
    /* Stack element	*/
    class StkNN(var a: DbvtNode? = null, var b: DbvtNode? = null)

    class StkNP(var node: DbvtNode? = null, var mask: Int = 0)
    class StkNPS(var node: DbvtNode? = null, var mask: Int = 0, var value: Float = 0f)
    class StkCLN(var node: DbvtNode? = null, var parent: DbvtNode? = null)

    // Policies/Interfaces

    interface Collide {
        fun process(leaf0: DbvtNode, leaf1: DbvtNode) = Unit
        infix fun process(node: DbvtNode) = Unit
        fun process(n: DbvtNode, s: Float) = process(n)
        infix fun descent(node: DbvtNode) = true
        infix fun allLeaves(node: DbvtNode?) = true
    }

    interface Writer {
        fun prepare(root: DbvtNode, numNodes: Int) = 0
        fun writeNode(node: DbvtNode, index: Int, parent: Int, child0: Int, child1: Int) = 0
        fun writeLeaf(node: DbvtNode, index: Int, parent: Int) = 0
    }

    interface Clone {
        fun cloneLeaf(node: DbvtNode)
    }

    // Fields
    var root: DbvtNode? = null
    var free: DbvtNode? = null
    var lkhd = -1
    var leaves = 0
    var opath = 0


    val stkStack = ArrayList<StkNN>()

    // Methods
    fun clear() {
        root?.let(::recurseDeleteNode)
        free = null
        lkhd = -1
        stkStack.clear()
        opath = 0
    }

    fun empty() = root == null
    fun optimizeBottomUp() {
        root?.let {
            val leaves = ArrayList<DbvtNode>()
            fetchLeaves(it, leaves)
            bottomUp(this, leaves, leaves.size)
            root = leaves[0]
        }
    }

    fun optimizeTopDown(buTreshold: Int = 128) {
        root?.let {
            val leaves = ArrayList<DbvtNode>()
            fetchLeaves(it, leaves)
            root = topDown(this, leaves, leaves.size, buTreshold)
        }
    }

    fun optimizeIncremental(passes: Int) {
        var passes = if (passes < 0) leaves else passes
        root?.let {
            if (passes > 0) {
                do {
                    var node = it
                    var bit = 0
                    while (node.isInternal) {
                        node = sort(stkStack, node, it).childs[(opath ushr bit) and 1]!!
                        bit = (bit + 1) and (Int.BYTES * 8 - 1)
                    }
                    update(node)
                    ++opath
                } while (--passes != 0)
            }
        }
    }

    fun insert(volume: DbvtVolume, data: Any?): DbvtNode {
        val leaf = createNode(null, volume, data)
        insertLeaf(::root, leaf)
        ++leaves
        return leaf
    }

    fun update(leaf: DbvtNode, lookAhead: Int = -1) {
        ref = removeLeaf(leaf)
        if (ref != null) {
            if (lookAhead >= 0) {
                var i = 0
                while (i < lookAhead) {
                    ref!!.parent?.let { ref = it }
                    ++i
                }
            } else ref = root
        }
        insertLeaf(::ref, leaf)
    }

    fun update(leaf: DbvtNode, volume: DbvtVolume) {
        ref = removeLeaf(leaf)
        if (ref != null)
            if (lkhd >= 0) {
                var i = 0
                while (i < lkhd) {
                    ref!!.parent?.let { ref = it }
                    ++i
                }
            } else ref = root
        leaf.volume put volume
        insertLeaf(::ref, leaf)
    }

    fun update(leaf: DbvtNode, volume: DbvtVolume, velocity: Vec3, margin: Float): Boolean {
        if (leaf.volume contain volume) return false
        volume expand margin
        volume signedExpand velocity
        update(leaf, volume)
        return true
    }

    fun update(leaf: DbvtNode, volume: DbvtVolume, velocity: Vec3): Boolean {
        if (leaf.volume contain volume) return false
        volume signedExpand velocity
        update(leaf, volume)
        return true
    }

    fun update(leaf: DbvtNode, volume: DbvtVolume, margin: Float): Boolean {
        if (leaf.volume contain volume) return false
        volume expand margin
        update(leaf, volume)
        return true
    }

    infix fun remove(leaf: DbvtNode?) {
        removeLeaf(leaf)
        deleteNode(leaf)
        --leaves
    }

    fun write(writer: Writer) {
        val nodes = DbvtNodeEnumerator()
        TODO()
//        nodes.nodes.reserve(leaves * 2)
//        enumNodes(root, nodes)
//        writer.prepare(root, nodes.nodes.size)
        for (i in 0 until nodes.nodes.size) {
            val n = nodes.nodes[i]
            val p = n.parent?.let { nodes.nodes.indexOf(it) } ?: -1
            if (n.isInternal) {
                val c0 = nodes.nodes.indexOf(n.childs[0])
                val c1 = nodes.nodes.indexOf(n.childs[1])
                writer.writeNode(n, i, p, c0, c1)
            } else
                writer.writeLeaf(n, i, p)
        }
    }

    fun clone(dest: Dbvt, clone: Clone? = null) {
        dest.clear()
        root?.let {
            val stack = ArrayList<StkCLN>(leaves)
            stack.add(StkCLN(it, null))
            do {
                val i = stack.lastIndex
                val e = stack[i]
                val n = dest.createNode(e.parent, e.node!!.volume, e.node!!.data)
                stack.pop()
                if (e.parent != null)
                    e.parent!!.childs[i and 1] = n
                else
                    dest.root = n
                if (e.node!!.isInternal) {
                    stack.add(StkCLN(e.node!!.childs[0], n))
                    stack.add(StkCLN(e.node!!.childs[1], n))
                } else
                    clone!!.cloneLeaf(n)
            } while (stack.isNotEmpty())
        }
    }


    //    #if DBVT_ENABLE_BENCHMARK
//    static void        benchmark()
//    #else
//    static void        benchmark()
//    {}
//    #endif
//    // DBVT_IPOLICY must support ICollide policy/interface
//    DBVT_PREFIX
//    static void        enumNodes(    const btDbvtNode* root,
//    DBVT_IPOLICY)
//    DBVT_PREFIX
//    static void        enumLeaves(    const btDbvtNode* root,
//    DBVT_IPOLICY)
//    DBVT_PREFIX
//    void        collideTT(    const btDbvtNode* root0,
//    const btDbvtNode* root1,
//    DBVT_IPOLICY)
//
    fun collideTTpersistentStack(root0: DbvtNode?, root1: DbvtNode?, collider: DbvtTreeCollider) {

        if (root0 != null && root1 != null) {
            var depth = 1
            var treshold = DOUBLE_STACKSIZE - 4

            val element = StkNN(root0, root1)
            if (stkStack.isNotEmpty()) stkStack[0] = element
            else stkStack += element
            for (i in stkStack.size until DOUBLE_STACKSIZE)
                stkStack += StkNN()
            do {
                val p = stkStack[--depth]
                if (depth > treshold) {
                    for (i in stkStack.size until stkStack.size * 2)
                        stkStack += StkNN()
                    treshold = stkStack.size - 4
                }
                val pa = p.a!!
                val pb = p.b!!
                if (pa === pb) {
                    if (pa.isInternal) {
                        stkStack[depth++] = StkNN(pa.childs[0], pa.childs[0])
                        stkStack[depth++] = StkNN(pa.childs[1], pa.childs[1])
                        stkStack[depth++] = StkNN(pa.childs[0], pa.childs[1])
                    }
                } else if (pa.volume intersect pb.volume)
                    if (pa.isInternal)
                        if (pb.isInternal) {
                            stkStack[depth++] = StkNN(pa.childs[0], pb.childs[0])
                            stkStack[depth++] = StkNN(pa.childs[1], pb.childs[0])
                            stkStack[depth++] = StkNN(pa.childs[0], pb.childs[1])
                            stkStack[depth++] = StkNN(pa.childs[1], pb.childs[1])
                        } else {
                            stkStack[depth++] = StkNN(pa.childs[0], pb)
                            stkStack[depth++] = StkNN(pa.childs[1], pb)
                        }
                    else
                        if (pb.isInternal) {
                            stkStack[depth++] = StkNN(pa, pb.childs[0])
                            stkStack[depth++] = StkNN(pa, pb.childs[1])
                        } else
                            collider.process(pa, pb)
            } while (depth != 0)
        }
    }

    //    #if 0
//    DBVT_PREFIX
//    void        collideTT(    const btDbvtNode* root0,
//    const btDbvtNode* root1,
//    const btTransform& xform,
//    DBVT_IPOLICY)
//    DBVT_PREFIX
//    void        collideTT(    const btDbvtNode* root0,
//    const btTransform& xform0,
//    const btDbvtNode* root1,
//    const btTransform& xform1,
//    DBVT_IPOLICY)
//    #endif
//
    fun collideTV(root: DbvtNode?, volume: DbvtVolume, collider: Dbvt.Collide) {
        if (root != null) {
            val stack = ArrayList<DbvtNode>(SIMPLE_STACKSIZE)
            stack push root
            do {
                val n = stack[stack.size - 1]
                stack.pop()
                if (n.volume intersect volume)
                    if (n.isInternal) {
                        stack push n.childs[0]!!
                        stack push n.childs[1]!!
                    } else
                        collider process n
            } while (stack.isNotEmpty())
        }
    }

    var _tMin = 1f
    /** rayTestInternal is faster than rayTest, because it uses a persistent stack (to reduce dynamic memory allocations
     *  to a minimum) and it uses precomputed signs/rayInverseDirections
     *  rayTestInternal is used by DbvtBroadphase to accelerate world ray casts */
    fun rayTestInternal(root: DbvtNode?, rayFrom: Vec3, rayTo: Vec3, rayDirectionInverse: Vec3, signs: IntArray,
                        lambdaMax: Float, aabbMin: Vec3, aabbMax: Vec3, stack: ArrayList<DbvtNode>, collider: Dbvt.Collide) {
        if (root != null) {
            var depth = 1
            var treshold = DOUBLE_STACKSIZE - 2
            stack resize DOUBLE_STACKSIZE
            stack[0] = root
            val bounds = Array(2, { Vec3() })
            do {
                val node = stack[--depth]
                bounds[0] = node.volume.min - aabbMax
                bounds[1] = node.volume.max - aabbMin
                _tMin = 1f
                val lambdaMin = 0f
                val result1 = rayAabb2(rayFrom, rayDirectionInverse, signs, bounds, ::_tMin, lambdaMin, lambdaMax)
                if (result1) {
                    if (node.isInternal) {
                        if (depth > treshold) {
                            stack resize (stack.size * 2)
                            treshold = stack.size - 2
                        }
                        stack[depth++] = node.childs[0]!!
                        stack[depth++] = node.childs[1]!!
                    } else
                        collider process node
                }
            } while (depth != 0)
        }
    }
//
//    DBVT_PREFIX
//    static void        collideKDOP(const btDbvtNode* root,
//    const btVector3* normals,
//    const btScalar* offsets,
//    int count,
//    DBVT_IPOLICY)
//    DBVT_PREFIX
//    static void        collideOCL(    const btDbvtNode* root,
//    const btVector3* normals,
//    const btScalar* offsets,
//    const btVector3& sortaxis,
//    int count,
//    DBVT_IPOLICY,
//    bool fullsort=true)
//    DBVT_PREFIX
//    static void        collideTU(    const btDbvtNode* root,
//    DBVT_IPOLICY)
//    // Helpers
//    static DBVT_INLINE int    nearest(const int* i,const btDbvt::sStkNPS* a,btScalar v,int l,int h)
//    {
//        int m =0
//        while (l < h) {
//            m = (l + h) > >1
//            if (a[i[m]].value >= v) l = m + 1; else h = m
//        }
//        return (h)
//    }
//    static DBVT_INLINE int    allocate(    btAlignedObjectArray<int>& ifree,
//    btAlignedObjectArray<sStkNPS>& stock,
//    const sStkNPS& value)
//    {
//        int i
//                if (ifree.size() > 0) {
//                    i = ifree[ifree.size() - 1];ifree.pop_back();stock[i] = value; } else {
//                    i = stock.size();stock.push_back(value); }
//        return (i)
//    }
//    //
//    private :
//    btDbvt(const btDbvt&)
//    {}

    fun collideTVNoStackAlloc(root: DbvtNode?, volume: DbvtVolume, stack: ArrayList<DbvtNode>, policy: Collide) {

        if (root != null) {
            stack.clear()
            stack resize SIMPLE_STACKSIZE
            stack.add(root)
            do {
                val n = stack.last()
                stack.pop()
                if (n.volume intersect volume)
                    if (n.isInternal) {
                        stack.add(n.childs[0]!!)
                        stack.add(n.childs[1]!!)
                    } else policy.process(n)
            } while (stack.isNotEmpty())
        }
    }

    // Constants
    companion object {

        var ref: DbvtNode? = null

        val SIMPLE_STACKSIZE = 64
        val DOUBLE_STACKSIZE = SIMPLE_STACKSIZE * 2

        fun maxDepth(node: DbvtNode?) = node?.let { node.getMaxDepth(1, 0) } ?: 0

        fun countLeaves(node: DbvtNode): Int = when {
            node.isInternal -> countLeaves(node.childs[0]!!) + countLeaves(node.childs[1]!!)
            else -> 1
        }

        fun extractLeaves(node: DbvtNode, leaves: ArrayList<DbvtNode>) {
            if (node.isInternal) {
                extractLeaves(node.childs[0]!!, leaves)
                extractLeaves(node.childs[1]!!, leaves)
            } else
                leaves.add(node)
        }

        var tMin = 1f
        var param = 1f

        /** rayTest is a re-entrant ray test, and can be called in parallel as long as the AlignedAlloc is thread-safe
         *  (uses locking etc)
         *  rayTest is slower than rayTestInternal, because it builds a local stack, using memory allocations, and it
         *  recomputes signs/rayDirectionInverses each time */
        fun rayTest(root: DbvtNode?, rayFrom: Vec3, rayTo: Vec3, policy: Collide) {
            if (root != null) {
                val rayDir = (rayTo - rayFrom).normalize()
                ///what about division by zero? --> just set rayDirection[i] to INF/BT_LARGE_FLOAT
                val rayDirectionInverse = Vec3(
                        if (rayDir[0] == 0f) LARGE_FLOAT else 1f / rayDir[0],
                        if (rayDir[1] == 0f) LARGE_FLOAT else 1f / rayDir[1],
                        if (rayDir[2] == 0f) LARGE_FLOAT else 1f / rayDir[2])
                val signs = IntArray(3, { (rayDirectionInverse[it] < 0f).i })
                val lambdaMax = rayDir dot (rayTo - rayFrom)
                val resultNormal = Vec3()
                val stack = ArrayList<DbvtNode>()
                var depth = 1
                var treshold = DOUBLE_STACKSIZE - 2
                stack.resize(DOUBLE_STACKSIZE)
                stack[0] = root
                val bounds = Array(2, { Vec3() })
                do {
                    val node = stack[--depth]

                    bounds[0] = node.volume.min
                    bounds[1] = node.volume.max

                    tMin = 1f
                    val lambdaMin = 0f
                    val result1 = rayAabb2(rayFrom, rayDirectionInverse, signs, bounds, ::tMin, lambdaMin, lambdaMax)

                    if (COMPARE_BTRAY_AABB2) {
                        param = 1f
                        val result2 = rayAabb(rayFrom, rayTo, node.volume.min, node.volume.max, ::param, resultNormal)
                        assert(result1 == result2)
                    }
                    if (result1) {
                        if (node.isInternal) {
                            if (depth > treshold) {
                                stack.resize(stack.size * 2)
                                treshold = stack.size - 2
                            }
                            stack[depth++] = node.childs[0]!!
                            stack[depth++] = node.childs[1]!!
                        } else
                            policy.process(node)
                    }
                } while (depth != 0)
            }
        }
    }

    infix fun deleteNode(node: DbvtNode?) {
        free = node
    }

    infix fun recurseDeleteNode(node: DbvtNode) {
        if (!node.isLeaf) {
            recurseDeleteNode(node.childs[0]!!)
            recurseDeleteNode(node.childs[1]!!)
        }
        if (node === root) root = null
        deleteNode(node)
    }

    fun createNode(parent: DbvtNode?, data: Any?): DbvtNode {
        val f = free
        return if (f != null) {
            free = null
            f
        } else
            DbvtNode().also {
                it.parent = parent
                it.data = data
                it.childs[1] = null
            }
    }

    fun createNode(parent: DbvtNode?, volume: DbvtVolume, data: Any?) = createNode(parent, data).also { it.volume put volume }
    fun createNode(parent: DbvtNode?, volume0: DbvtVolume, volume1: DbvtVolume, data: Any?) =
            createNode(parent, data).also { volume0.merge(volume1, it.volume) }

    fun insertLeaf(_root: KMutableProperty0<DbvtNode?>, leaf: DbvtNode) {
        if (root == null) {
            root = leaf
            leaf.parent = null
        } else {
            _root as KMutableProperty0<DbvtNode>
            if (!_root().isLeaf)
                do {
                    _root.set(_root().childs[leaf.volume.select(_root().childs[0]!!.volume, _root().childs[1]!!.volume)]!!)
                } while (!_root().isLeaf)
            var prev = _root().parent
            var node = createNode(prev, leaf.volume, _root().volume, 0)
            if (prev != null) {
                prev.childs[_root().indexOf] = node
                node.childs[0] = _root(); _root().parent = node
                node.childs[1] = leaf; leaf.parent = node
                do {
                    if (!prev!!.volume.contain(node.volume))
                        prev.childs[0]!!.volume.merge(prev.childs[1]!!.volume, prev.volume)
                    else break
                    node = prev
                    prev = node.parent
                } while (prev != null)
            } else {
                node.childs[0] = _root(); _root().parent = node
                node.childs[1] = leaf; leaf.parent = node
                root = node
            }
        }
    }

    fun removeLeaf(leaf: DbvtNode?) = when {
        leaf === root -> {
            root = null
            null
        }
        else -> {
            val parent = leaf!!.parent!!
            var prev = parent.parent
            val sibling = parent.childs[1 - leaf.indexOf]!!
            if (prev != null) {
                prev.childs[parent.indexOf] = sibling
                sibling.parent = prev
                deleteNode(parent)
                while (prev != null) {
                    val pb = prev.volume
                    prev.childs[0]!!.volume.merge(prev.childs[1]!!.volume, prev.volume)
                    if (pb != prev.volume)
                        prev = prev.parent
                    else break
                }
                prev ?: root
            } else {
                root = sibling
                sibling.parent = null
                deleteNode(parent)
                root
            }
        }
    }

    fun fetchLeaves(root: DbvtNode, leaves: ArrayList<DbvtNode>, depth: Int = -1) {
        if (root.isInternal && depth != 0) {
            fetchLeaves(root.childs[0]!!, leaves, depth - 1)
            fetchLeaves(root.childs[1]!!, leaves, depth - 1)
            deleteNode(root)
        } else leaves push root
    }
}

class DbvtNodeEnumerator : Dbvt.Collide {
    var nodes = ArrayList<DbvtNode>()
    override fun process(node: DbvtNode) {
        nodes.add(node)
    }
}

fun leftOfAxis(node: DbvtNode, org: Vec3, axis: Vec3) = (axis dot (node.volume.center - org)) <= 0

/** Partitions leaves such that leaves[0, n) are on the left of axis, and leaves[n, count) are on the right of axis.
 *  returns N.  */
fun split(leaves: ArrayList<DbvtNode>, count: Int, org: Vec3, axis: Vec3) = split(leaves, 0, count, org, axis)

fun split(leaves: ArrayList<DbvtNode>, ptr: Int, count: Int, org: Vec3, axis: Vec3): Int {
    var begin = 0
    var end = count
    while (true) {
        while (begin != end && leftOfAxis(leaves[ptr + begin], org, axis))
            ++begin
        if (begin == end) break
        while (begin != end && !leftOfAxis(leaves[ptr + end - 1], org, axis))
            --end
        if (begin == end) break
        // swap out of place nodes
        --end
        val temp = leaves[ptr + begin]
        leaves[ptr + begin] = leaves[ptr + end]
        leaves[ptr + end] = temp
        ++begin
    }
    return begin
}

fun bounds(leaves: ArrayList<DbvtNode>, count: Int) = bounds(leaves, 0, count)
fun bounds(leaves: ArrayList<DbvtNode>, ptr: Int, count: Int): DbvtVolume {
    val volume = leaves[ptr].volume
    var i = 1
    while (i < count) {
        volume.merge(leaves[ptr + i].volume, volume)
        ++i
    }
    return volume
}

fun bottomUp(dbvt: Dbvt, leaves: ArrayList<DbvtNode>, ptr: Int, count: Int = 0) {
    var count = count
    while (count > 1) {
        var minsize = Float.MAX_VALUE
        val minIdx = intArrayOf(-1, -1)
        for (i in 0 until count)
            for (j in i + 1 until count) {
                val sz = (leaves[ptr + i].volume merge leaves[ptr + j].volume).size
                if (sz < minsize) {
                    minsize = sz
                    minIdx[0] = i
                    minIdx[1] = j
                }
            }
        val n = arrayOf(leaves[ptr + minIdx[0]], leaves[ptr + minIdx[1]])
        val p = dbvt.createNode(null, n[0].volume, n[1].volume, 0)
        p.childs[0] = n[0]
        p.childs[1] = n[1]
        n[0].parent = p
        n[1].parent = p
        leaves[ptr + minIdx[0]] = p
        leaves[ptr + minIdx[1]] = leaves[count - 1]
        --count
    }
}

private val axis = arrayOf(Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1))

fun topDown(dbvt: Dbvt, leaves: ArrayList<DbvtNode>, count: Int, buTreshold: Int) =
        topDown(dbvt, leaves, 0, count, buTreshold)

fun topDown(dbvt: Dbvt, leaves: ArrayList<DbvtNode>, ptr: Int, count: Int, buTreshold: Int): DbvtNode {
    assert(buTreshold > 2)
    if (count > 1) {
        if (count > buTreshold) {
            val vol = bounds(leaves, ptr, count)
            val org = vol.center
            val partition: Int
            var bestAxis = -1
            var bestMidp = count
            val splitcount = arrayOf(intArrayOf(0, 0), intArrayOf(0, 0), intArrayOf(0, 0))
            var i = 0
            while (i < count) {
                val x = leaves[ptr + i].volume.center - org
                for (j in 0..2)
                    ++splitcount[j][if ((x dot axis[j]) > 0) 1 else 0]
                ++i
            }
            i = 0
            while (i < 3) {
                if (splitcount[i][0] > 0 && splitcount[i][1] > 0) {
                    val midp = abs(splitcount[i][0] - splitcount[i][1])
                    if (midp < bestMidp) {
                        bestAxis = i
                        bestMidp = midp
                    }
                }
                ++i
            }
            if (bestAxis >= 0) {
                partition = split(leaves, ptr, count, org, axis[bestAxis])
                assert(partition != 0 && partition != count)
            } else
                partition = count / 2 + 1
            val node = dbvt.createNode(null, vol, 0)
            node.childs[0] = topDown(dbvt, leaves, 0, partition, buTreshold)
            node.childs[1] = topDown(dbvt, leaves, partition, count - partition, buTreshold)
            node.childs[0]!!.parent = node
            node.childs[1]!!.parent = node
            return node
        } else {
            bottomUp(dbvt, leaves, ptr, count)
            return leaves[ptr]
        }
    }
    return leaves[ptr]
}

fun sort(stack: ArrayList<Dbvt.StkNN>, n: DbvtNode, r: DbvtNode): DbvtNode {
    val p = n.parent
    assert(n.isInternal)
    if (p != null) {
        TODO()
//        if (p > n) {
//            const int i = indexof(n)
//            const int j = 1 - i
//            btDbvtNode * s = p->childs[j]
//            btDbvtNode * q = p->parent
//            btAssert(n == p->childs[i])
//            if (q) q->childs[indexof(p)] = n; else r = n
//            s->parent = n
//            p->parent = n
//            n->parent = q
//            p->childs[0] = n->childs[0]
//            p->childs[1] = n->childs[1]
//            n->childs[0]->parent = p
//            n->childs[1]->parent = p
//            n->childs[i] = p
//            n->childs[j] = s
//            btSwap(p->volume, n->volume)
//            return (p)
//        }
    }
    return n
}