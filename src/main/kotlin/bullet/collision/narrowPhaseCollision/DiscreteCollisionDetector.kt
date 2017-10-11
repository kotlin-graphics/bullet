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

package bullet.collision.narrowPhaseCollision

import bullet.linearMath.DebugDraw
import bullet.linearMath.LARGE_FLOAT
import bullet.linearMath.Transform
import bullet.linearMath.Vec3

/** This interface is made to be used by an iterative approach to do TimeOfImpact calculations
 *  This interface allows to query for closest points and penetration depth between two (convex) objects
 *  The closest point is on the second object (B), and the normal points from the surface on B towards A.
 *  Distance is between closest points on B and closest point on A. So you can calculate closest point on A by taking
 *  closestPointInA = closestPointInB + distance * normalOnSurfaceB */
interface DiscreteCollisionDetectorInterface {

    interface Result {

        /** provides experimental support for per-triangle material / custom material combiner  */
        fun setShapeIdentifiersA(partId0: Int, index0: Int)

        /** provides experimental support for per-triangle material / custom material combiner  */
        fun setShapeIdentifiersB(partId1: Int, index1: Int)

        fun addContactPoint(normalOnBInWorld: Vec3, pointInWorld: Vec3, depth: Float)
    }

    class ClosestPointInput {
        val transformA = Transform()
        val transformB = Transform()
        var maximumDistanceSquared = LARGE_FLOAT
    }

    /** give either closest points (distance > 0) or penetration (distance)
     *  the normal always points from B towards A   */
    fun getClosestPoints(input: ClosestPointInput, output: Result, debugDraw: DebugDraw?, swapResults: Boolean = false)
}

abstract class StorageResult : DiscreteCollisionDetectorInterface.Result {

    val normalOnSurfaceB = Vec3()
    val closestPointInB = Vec3()
    /** negative means penetration !    */
    var distance = LARGE_FLOAT

    override fun addContactPoint(normalOnBInWorld: Vec3, pointInWorld: Vec3, depth: Float) {
        if (depth < distance) {
            normalOnSurfaceB put normalOnBInWorld
            closestPointInB put pointInWorld
            distance = depth
        }
    }
}