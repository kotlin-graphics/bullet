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

import bullet.collision.collisionDispatch.CollisionObject
import bullet.linearMath.Vec3

interface BroadphaseAabbCallback {
    infix fun process(proxy: BroadphaseProxy) = false
}


open class BroadphaseRayCallback : BroadphaseAabbCallback {
    /** added some cached data to accelerate ray-AABB tests */
    val rayDirectionInverse = Vec3()
    val signs = IntArray(3)
    var lambdaMax = 0f
}

/** The BroadphaseInterface class provides an interface to detect aabb-overlapping object pairs.
 *  Some implementations for this broadphase interface include AxisSweep3, _32BitAxisSweep3 and DbvtBroadphase.
 *  The actual overlapping pair management, storage, adding and removing of pairs is dealt by the OverlappingPairCache
 *  class.  */
interface BroadphaseInterface {

    fun createProxy(aabbMin: Vec3, aabbMax: Vec3, shapeType: Int, userPtr: CollisionObject, collisionFilterGroup: Int,
                    collisionFilterMask: Int, dispatcher: Dispatcher): BroadphaseProxy

    fun destroyProxy(proxy: BroadphaseProxy, dispatcher: Dispatcher)
    fun setAabb(proxy: BroadphaseProxy, aabbMin: Vec3, aabbMax: Vec3, dispatcher: Dispatcher)
    fun getAabb(proxy: BroadphaseProxy, aabbMin: Vec3, aabbMax: Vec3)

    fun rayTest(rayFrom: Vec3, rayTo: Vec3, rayCallback: BroadphaseRayCallback, aabbMin: Vec3 = Vec3(), aabbMax: Vec3 = Vec3())

    fun aabbTest(aabbMin: Vec3, aabbMax: Vec3, callback: BroadphaseAabbCallback)

    /** calculateOverlappingPairs is optional: incremental algorithms (sweep and prune) might do it during the set aabb */
    fun calculateOverlappingPairs(dispatcher: Dispatcher)

    val overlappingPairCache: OverlappingPairCache

    /** getAabb returns the axis aligned bounding box in the 'global' coordinate frame will add some transform later    */
    fun getBroadphaseAabb(aabbMin: Vec3, aabbMax: Vec3)

    /** reset broadphase internal structures, to ensure determinism/reproducability */
    fun resetPool(dispatcher: Dispatcher)

    fun printStats()
}
