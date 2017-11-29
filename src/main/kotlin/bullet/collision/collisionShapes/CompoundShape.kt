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
import bullet.collision.broadphaseCollision.Dbvt
import bullet.collision.broadphaseCollision.DbvtNode
import bullet.collision.broadphaseCollision.DbvtVolume
import bullet.linearMath.*
import bullet.pop
import bullet.swap
import bullet.swapLastAt

class CompoundShapeChild {
    val transform = Transform()
    lateinit var childShape: CollisionShape
    lateinit var childShapeType: BroadphaseNativeTypes
    var childMargin = 0f
    lateinit var node: DbvtNode
    override fun equals(other: Any?) = other is CompoundShapeChild && transform == other.transform &&
            childShape == other.childShape && childShapeType == other.childShapeType && childMargin == other.childMargin

    override fun hashCode(): Int {
        var result = 31 * transform.hashCode() + childShape.hashCode()
        result = 31 * result + childShapeType.hashCode()
        result = 31 * result + childMargin.hashCode()
        return 31 * result + node.hashCode()
    }
}

/** The CompoundShape allows to store multiple other CollisionShapes
 *  This allows for moving concave collision objects. This is more general then the static concave BvhTriangleMeshShape.
 *  It has an (optional) dynamic aabb tree to accelerate early rejection tests.
 *  @todo: This aabb tree can also be use to speed up ray tests on CompoundShape,
 *  see http://code.google.com/p/bullet/issues/detail?id=25
 *  Currently, removal of child shapes is only supported when disabling the aabb tree (pass 'false' in the constructor
 *  of CompoundShape) */
class CompoundShape(enableDynamicAabbTree: Boolean = true, initialChildCapacity: Int = 0) : CollisionShape() {

    val children = ArrayList<CompoundShapeChild>(initialChildCapacity)
    val localAabbMin = Vec3(LARGE_FLOAT)
    val localAabbMax = Vec3(-LARGE_FLOAT)

    var dynamicAabbTree: Dbvt? = null
    /** increment m_updateRevision when adding/removing/replacing child shapes, so that some caches can be updated */
    var updateRevision = 1
    var collisionMargin = 0f
    override var localScaling = Vec3(1f) // TODO check
        set(value) {
            for (i in 0 until children.size) {
                val childTrans = getChildTransform(i)
                val childScale = children[i].childShape.localScaling * value / localScaling
                children[i].childShape.localScaling = childScale
                childTrans.origin = childTrans.origin * value / localScaling
                updateChildTransform(i, childTrans, false)
            }
            field put value
            recalculateLocalAabb()
        }

    init {
        shapeType = BroadphaseNativeTypes.COMPOUND_SHAPE_PROXYTYPE
        if (enableDynamicAabbTree) dynamicAabbTree = Dbvt()
    }

    fun addChildShape(localTransform: Transform, shape: CollisionShape) {
        updateRevision++
        val child = CompoundShapeChild().apply {
            transform put localTransform
            childShape = shape
            childShapeType = shape.shapeType
            childMargin = shape.margin
        }
        //extend the local aabbMin/aabbMax
        val localAabbMin = Vec3()
        val localAabbMax = Vec3()
        shape.getAabb(localTransform, localAabbMin, localAabbMax)
        for (i in 0..2) {
            if (this.localAabbMin[i] > localAabbMin[i]) this.localAabbMin[i] = localAabbMin[i]
            if (this.localAabbMax[i] < localAabbMax[i]) this.localAabbMax[i] = localAabbMax[i]
        }
        dynamicAabbTree?.let {
            val bounds = DbvtVolume.fromMM(localAabbMin, localAabbMax)
            val index = children.size
            child.node = it.insert(bounds, index)
        }
        children.add(child)
    }

    /** Remove all children shapes that contain the specified shape */
    fun removeChildShape(shape: CollisionShape) {
        updateRevision++
        /*  Find the children containing the shape specified, and remove those children.
            note: there might be multiple children using the same shape!         */
        for (i in children.size - 1 downTo 0)
            if (children[i].childShape == shape)
                removeChildShapeByIndex(i)
        recalculateLocalAabb()
    }

    fun removeChildShapeByIndex(childShapeIndex: Int) {
        updateRevision++
        assert(childShapeIndex >= 0 && childShapeIndex < children.size)
        dynamicAabbTree?.remove(children[childShapeIndex].node)
        children.swapLastAt(childShapeIndex)
        dynamicAabbTree?.let { children[childShapeIndex].node.dataAsInt = childShapeIndex }
        children.pop()
    }

    val numChildShapes get() = children.size
    fun getChildShape(index: Int) = children[index].childShape
    fun getChildTransform(index: Int) = children[index].transform

    /** set a new transform for a child, and update internal data structures (local aabb and dynamic tree) */
    fun updateChildTransform(childIndex: Int, newChildTransform: Transform, shouldRecalculateLocalAabb: Boolean = true) {
        children[childIndex].transform put newChildTransform
        dynamicAabbTree?.let {
            // update the dynamic aabb tree
            val localAabbMin = Vec3()
            val localAabbMax = Vec3()
            children[childIndex].childShape.getAabb(newChildTransform, localAabbMin, localAabbMax)
            val bounds = DbvtVolume.fromMM(localAabbMin, localAabbMax)
            //int index = m_children.size()-1;
            it.update(children[childIndex].node, bounds)
        }
        if (shouldRecalculateLocalAabb) recalculateLocalAabb()
    }

    val childList get() = children

    /** getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version */
    override fun getAabb(trans: Transform, aabbMin: Vec3, aabbMax: Vec3) {
        val localHalfExtents = 0.5f * (localAabbMax - localAabbMin)
        val localCenter = 0.5f * (localAabbMax + localAabbMin)
        //avoid an illegal AABB when there are no children
        if (children.isEmpty()) {
            localHalfExtents put 0f
            localCenter put 0f
        }
        localHalfExtents += margin
        val absB = trans.basis.absolute()
        val center = trans(localCenter)
        val extent = localHalfExtents dot3 absB
        aabbMin put center - extent // TODO check
        aabbMax put center + extent
    }

    /** Re-calculate the local Aabb. Is called at the end of removeChildShapes.
    Use this yourself if you modify the children or their transforms. */
    fun recalculateLocalAabb() {
        // Recalculate the local aabb
        // Brute force, it iterates over all the shapes left.
        localAabbMin put LARGE_FLOAT
        localAabbMax put -LARGE_FLOAT
        //extend the local aabbMin/aabbMax
        children.forEach {
            val localAabbMin = Vec3()
            val localAabbMax = Vec3()
            it.childShape.getAabb(it.transform, localAabbMin, localAabbMax)
            for (i in 0..2) {
                if (this.localAabbMin[i] > localAabbMin[i]) this.localAabbMin[i] = localAabbMin[i]
                if (this.localAabbMax[i] < localAabbMax[i]) this.localAabbMax[i] = localAabbMax[i]
            }
        }
    }

    override fun calculateLocalInertia(mass: Float, inertia: Vec3) {
        //approximation: take the inertia from the aabb for now
        val ident = Transform().apply { setIdentity() }
        val aabbMin = Vec3()
        val aabbMax = Vec3()
        getAabb(ident, aabbMin, aabbMax)
        val halfExtents = (aabbMax - aabbMin) * 0.5f
        val lx = 2f * halfExtents.x
        val ly = 2f * halfExtents.y
        val lz = 2f * halfExtents.z
        inertia[0] = mass / 12f * (ly * ly + lz * lz)
        inertia[1] = mass / 12f * (lx * lx + lz * lz)
        inertia[2] = mass / 12f * (lx * lx + ly * ly)
    }

    override var margin
    set(value) {
        collisionMargin = value
    }
    get() = collisionMargin

    override val name get() = "Compound"

    fun createAabbTreeFromChildren() {
        if (dynamicAabbTree == null) {
            dynamicAabbTree = Dbvt ()
            for (index in 0 until children.size)            {
                val child = children[index]
                // extend the local aabbMin/aabbMax
                val localAabbMin = Vec3()
                val localAabbMax = Vec3()
                child.childShape.getAabb(child.transform, localAabbMin, localAabbMax)
                val bounds = DbvtVolume.fromMM(localAabbMin, localAabbMax)
                child.node = dynamicAabbTree!!.insert(bounds, index)
            }
        }
    }

    /** Computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes
     *  of the moment of inertia and the center of mass to the current coordinate system. "masses" points to an array of
     *  masses of the children. The resulting transform "principal" has to be applied inversely to all children
     *  transforms in order for the local coordinate system of the compound shape to be centered at the center of mass
     *  and to coincide with the principal axes. This also necessitates a correction of the world transform of the
     *  collision object by the principal transform. */
    fun calculatePrincipalAxisTransform(masses: FloatArray, principal: Transform, inertia: Vec3) {
        val n = children.size
        var totalMass = 0f
        val center = Vec3()
        for (k in 0 until n) {
            assert(masses[k] > 0)
            center += children[k].transform.origin * masses[k]
            totalMass += masses[k]
        }
        assert(totalMass > 0)
        center /= totalMass
        principal.origin = center
        val tensor = Mat3()
        for (k in 0 until n) {
            val i = Vec3()
            children[k].childShape.calculateLocalInertia(masses[k], i)
            val t = children[k].transform
            val o = t.origin - center
            //compute inertia tensor in coordinate system of compound shape
            var j = t.basis.transpose()
            j[0] *= i[0]
            j[1] *= i[1]
            j[2] *= i[2]
            j = t.basis * j
            //add inertia tensor
            tensor[0] += j[0]
            tensor[1] += j[1]
            tensor[2] += j[2]
            //compute inertia tensor of pointmass at o
            val o2 = o.length2()
            j[0].put(o2, 0, 0)
            j[1].put(0, o2, 0)
            j[2].put(0, 0, o2)
            j[0] += o * -o.x
            j[1] += o * -o.y
            j[2] += o * -o.z
            //add inertia tensor of pointmass
            tensor[0] += masses[k] * j[0]
            tensor[1] += masses[k] * j[1]
            tensor[2] += masses[k] * j[2]
        }
        tensor.diagonalize(principal.basis, 0.00001f, 20)
        inertia.put(tensor[0][0], tensor[1][1], tensor[2][2])
    }
}
