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

package bullet.collision.broadphaseCollision

import bullet.collision.broadphaseCollision.BroadphaseNativeTypes.*
import bullet.linearMath.Vec3

/** Dispatcher uses these types
 *  IMPORTANT NOTE:The types are ordered polyhedral, implicit convex and concave to facilitate type checking
 *  CUSTOM_POLYHEDRAL_SHAPE_TYPE,CUSTOM_CONVEX_SHAPE_TYPE and CUSTOM_CONCAVE_SHAPE_TYPE can be used to extend Bullet
 *  without modifying source code   */
enum class BroadphaseNativeTypes {
    // polyhedral convex shapes
    BOX_SHAPE_PROXYTYPE,
    TRIANGLE_SHAPE_PROXYTYPE,
    TETRAHEDRAL_SHAPE_PROXYTYPE,
    CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE,
    CONVEX_HULL_SHAPE_PROXYTYPE,
    CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE,
    CUSTOM_POLYHEDRAL_SHAPE_TYPE,
    //implicit convex shapes
    IMPLICIT_CONVEX_SHAPES_START_HERE,
    SPHERE_SHAPE_PROXYTYPE,
    MULTI_SPHERE_SHAPE_PROXYTYPE,
    CAPSULE_SHAPE_PROXYTYPE,
    CONE_SHAPE_PROXYTYPE,
    CONVEX_SHAPE_PROXYTYPE,
    CYLINDER_SHAPE_PROXYTYPE,
    UNIFORM_SCALING_SHAPE_PROXYTYPE,
    MINKOWSKI_SUM_SHAPE_PROXYTYPE,
    MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE,
    BOX_2D_SHAPE_PROXYTYPE,
    CONVEX_2D_SHAPE_PROXYTYPE,
    CUSTOM_CONVEX_SHAPE_TYPE,
    //concave shapes
    CONCAVE_SHAPES_START_HERE,
    //keep all the convex shapetype below here, for the check IsConvexShape in broadphase proxy!
    TRIANGLE_MESH_SHAPE_PROXYTYPE,
    SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE,
    ///used for demo integration FAST/Swift collision library and Bullet
    FAST_CONCAVE_MESH_PROXYTYPE,
    //terrain
    TERRAIN_SHAPE_PROXYTYPE,
    ///Used for GIMPACT Trimesh integration
    GIMPACT_SHAPE_PROXYTYPE,
    ///Multimaterial mesh
    MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE,

    EMPTY_SHAPE_PROXYTYPE,
    STATIC_PLANE_PROXYTYPE,
    CUSTOM_CONCAVE_SHAPE_TYPE,
    CONCAVE_SHAPES_END_HERE,

    COMPOUND_SHAPE_PROXYTYPE,

    SOFTBODY_SHAPE_PROXYTYPE,
    HFFLUID_SHAPE_PROXYTYPE,
    HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE,
    INVALID_SHAPE_PROXYTYPE,

    MAX_BROADPHASE_COLLISION_TYPES;

    val i = ordinal
}

/** The BroadphaseProxy is the main class that can be used with the Bullet broadphases.
 *  It stores collision shape type information, collision filter information and a client object, typically a
 *  CollisionObject or RigidBody.   */
class BroadphaseProxy {

    /** optional filtering to cull potential collisions */
    enum class CollisionFilterGroups(val i: Int) {
        DefaultFilter(1),
        StaticFilter(2),
        KinematicFilter(4),
        DebrisFilter(8),
        SensorTrigger(16),
        CharacterFilter(32),
        /** all bits sets: DefaultFilter | StaticFilter | KinematicFilter | DebrisFilter | SensorTrigger    */
        AllFilter(-1)
    }

    /** Usually the client btCollisionObject or Rigidbody class */
//    void*    m_clientObject;
    var collisionFilterGroup = 0
    var collisionFilterMask = 0

    /** uniqueId is introduced for paircache. could get rid of this, by calculating the address offset etc. */
    var uniqueId = 0

    var aabbMin = Vec3()
    var aabbMax = Vec3()

    //used for memory pools
//    btBroadphaseProxy() :m_clientObject(0)    {    }

    constructor()

    constructor(aabbMin: Vec3, aabbMax: Vec3, userPtr: Any?, collisionFilterGroup: Int, collisionFilterMask: Int) {
//    :m_clientObject(userPtr), TODO
        this.collisionFilterGroup = collisionFilterGroup
        this.collisionFilterMask = collisionFilterMask
        aabbMin put aabbMin
        aabbMax put aabbMax
    }

    companion object {

        fun isPolyhedral(proxyType: Int) = proxyType < IMPLICIT_CONVEX_SHAPES_START_HERE.i
        fun isConvex(proxyType: Int) = proxyType < CONCAVE_SHAPES_START_HERE.i
        fun isNonMoving(proxyType: Int) = isConcave(proxyType) && proxyType != GIMPACT_SHAPE_PROXYTYPE.i
        fun isConcave(proxyType: Int) = proxyType > CONCAVE_SHAPES_START_HERE.i && proxyType < CONCAVE_SHAPES_END_HERE.i
        fun isCompound(proxyType: Int) = proxyType == COMPOUND_SHAPE_PROXYTYPE.i
        fun isSoftBody(proxyType: Int) = proxyType == SOFTBODY_SHAPE_PROXYTYPE.i
        fun isInfinite(proxyType: Int) = proxyType == STATIC_PLANE_PROXYTYPE.i
        fun isConvex2d(proxyType: Int) = proxyType == BOX_2D_SHAPE_PROXYTYPE.i || proxyType == CONVEX_2D_SHAPE_PROXYTYPE.i
    }
}

/** The BroadphasePair class contains a pair of aabb-overlapping objects.
 *  A Dispatcher can search a CollisionAlgorithm that performs exact/narrowphase collision detection
 *  on the actual collision shapes. */
class BroadphasePair {

    var proxy0: BroadphaseProxy? = null
    var proxy1: BroadphaseProxy? = null

    var algorithm: CollisionAlgorithm? = null
    /** don't use this data, it will be removed in future version.  */
    var internalInfo1: Any? = null

    constructor()
    constructor(other: BroadphasePair) {
        proxy0 = other.proxy0
        proxy1 = other.proxy1
        algorithm = other.algorithm
        internalInfo1 = other.internalInfo1
    }
    constructor(proxy0:BroadphaseProxy,proxy1:BroadphaseProxy)    {
        //keep them sorted, so the std::set operations work
        if (proxy0.uniqueId < proxy1.uniqueId) {
            this.proxy0 = proxy0
            this.proxy1 = proxy1
        } else {
            this.proxy0 = proxy1
            this.proxy1 = proxy0
        }
        algorithm = null
        internalInfo1 = null
    }

    override fun equals(other: Any?) = other is BroadphasePair && proxy0 === other.proxy0 && proxy1 == other.proxy1
    override fun hashCode() = 31 * (proxy0?.hashCode() ?: 0) + (proxy1?.hashCode() ?: 0)
}
