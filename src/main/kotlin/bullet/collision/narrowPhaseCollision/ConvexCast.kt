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


/** ConvexCast is an interface for Casting    */
abstract class ConvexCast {

    /** RayResult stores the closest result
     *  alternatively, add a callback method to decide about closest/all results    */
    class CastResult {
        infix fun debugDraw(fraction: Float) = Unit
        infix fun drawCoordSystem(trans: Transform) = Unit
        fun reportFailure(errNo: Int, numIterations: Int) = Unit

        val hitTransformA = Transform()
        val hitTransformB = Transform()
        val normal = Vec3()
        val hitPoint = Vec3()
        /** input and output    */
        var fraction = LARGE_FLOAT
        var debugDrawer: DebugDraw? = null
        var allowedPenetration = 0f
    }

    /** cast a convex against another convex object */
    abstract fun calcTimeOfImpact(fromA: Transform, toA: Transform, fromB: Transform, toB: Transform, result: CastResult): Boolean
}
