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

import bullet.collision.collisionShapes.ConvexShape
import bullet.linearMath.Transform
import bullet.linearMath.Vec3

private val MAX_ITERATIONS = 32

/** GjkConvexCast performs a raycast on a convex object using support mapping.  */
class GjkConvexCast(val convexA: ConvexShape, val convexB: ConvexShape, val simplexSolver: SimplexSolverInterface) : ConvexCast() {

    override fun calcTimeOfImpact(fromA: Transform, toA: Transform, fromB: Transform, toB: Transform, result: CastResult): Boolean {

        simplexSolver.reset()

        /*  Compute linear velocity for this interval, to interpolate
            assume no rotation/angular velocity, assert here?         */
        val linVelA = toA.origin - fromA.origin
        val linVelB = toB.origin - fromB.origin

        val radius = 0.001f
        var lambda = 0f
        val v = Vec3(1f, 0f, 0f)

        var maxIter = MAX_ITERATIONS

        val n = Vec3()
        var hasResult = false
        val r = linVelA - linVelB

        var lastLambda = lambda
        //btScalar epsilon = btScalar(0.001);

        var numIter = 0
        //first solution, using GJK

        val identityTrans = Transform().apply { setIdentity() }

//	result.drawCoordSystem(sphereTr);

        val pointCollector = PointCollector()

        val gjk = GjkPairDetector(convexA, convexB, simplexSolver, null)//m_penetrationDepthSolver);
        val input = DiscreteCollisionDetectorInterface.ClosestPointInput()

        //we don't use margins during CCD
        //	gjk.setIgnoreMargin(true);

        input.transformA put fromA
        input.transformB put fromB
        gjk.getClosestPoints(input, pointCollector, null)

        hasResult = pointCollector.hasResult
        val c = Vec3(pointCollector.pointInWorld)

        if (hasResult) {
            var dist = pointCollector.distance
            n put pointCollector.normalOnBInWorld

            //not close enough
            while (dist > radius) {
                numIter++
                if (numIter > maxIter) return false //todo: report a failure
                var dLambda = 0f

                val projectedLinearVelocity = r dot n

                dLambda = dist / (projectedLinearVelocity)

                lambda -= dLambda

                if (lambda > 1f) return false

                if (lambda < 0f) return false

                //todo: next check with relative epsilon
                if (lambda <= lastLambda) {
                    return false
                    //n.setValue(0,0,0);
                    break
                }
                lastLambda = lambda

                //interpolate to next lambda
                result debugDraw lambda
                input.transformA.origin.setInterpolate3(fromA.origin, toA.origin, lambda)
                input.transformB.origin.setInterpolate3(fromB.origin, toB.origin, lambda)

                gjk.getClosestPoints(input, pointCollector, null)
                if (pointCollector.hasResult) {
                    if (pointCollector.distance < 0f) {
                        result.fraction = lastLambda
                        n put pointCollector.normalOnBInWorld
                        result.normal put n
                        result.hitPoint put pointCollector.pointInWorld
                        return true
                    }
                    c put pointCollector.pointInWorld
                    n put pointCollector.normalOnBInWorld
                    dist = pointCollector.distance
                } else return false //??
            }

            // is n normalized?
            // don't report time of impact for motion away from the contact normal (or causes minor penetration)
            if (n.dot(r) >= -result.allowedPenetration) return false

            result.fraction = lambda
            result.normal put n
            result.hitPoint put c
            return true
        }
        return false
    }
}