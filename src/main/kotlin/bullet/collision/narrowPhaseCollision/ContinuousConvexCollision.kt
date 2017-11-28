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

import bullet.EPSILON
import bullet.collision.collisionShapes.ConvexShape
import bullet.collision.collisionShapes.StaticPlaneShape
import bullet.f
import bullet.linearMath.Transform
import bullet.linearMath.TransformUtil
import bullet.linearMath.Vec3
import bullet.linearMath.times

/** This maximum should not be necessary. It allows for untested/degenerate cases in production code.
 *  You don't want your game ever to lock-up.   */
private val MAX_ITERATIONS = 64

/** ContinuousConvexCollision implements angular and linear time of impact for convex objects.
 *  Based on Brian Mirtich's Conservative Advancement idea (PhD thesis).
 *  Algorithm operates in worldspace, in order to keep in between motion globally consistent.
 *  It uses GJK at the moment. Future improvement would use minkowski sum / supporting vertex, merging innerloops   */
class ContinuousConvexCollision : ConvexCast {

    val convexA: ConvexShape
    /** second object is either a convex or a plane (code sharing)  */
    var convexB1: ConvexShape? = null
    var simplexSolver: SimplexSolverInterface? = null
    var penetrationDepthSolver: ConvexPenetrationDepthSolver? = null
    var planeShape: StaticPlaneShape? = null

    constructor(shapeA: ConvexShape, shapeB: ConvexShape, simplexSolver: SimplexSolverInterface?, penetrationDepthSolver: ConvexPenetrationDepthSolver) {
        convexA = shapeA
        convexB1 = shapeB
        this.simplexSolver = simplexSolver
        this.penetrationDepthSolver = penetrationDepthSolver
    }

    constructor(shapeA: ConvexShape, plane: StaticPlaneShape?) {
        convexA = shapeA
        planeShape = plane
    }

    fun computeClosestPoints(transA: Transform, transB: Transform, pointCollector: PointCollector) {
        val convexB = convexB1
        if (convexB != null) {
            simplexSolver!!.reset()
            val gjk = GjkPairDetector(convexA, convexB, convexA.shapeType.i, convexB.shapeType.i, convexA.margin,
                    convexB.margin, simplexSolver, penetrationDepthSolver)
            val input = DiscreteCollisionDetectorInterface.ClosestPointInput()
            input.transformA put transA
            input.transformB put transB
            gjk.getClosestPoints(input, pointCollector, null)
        } else {
            //convex versus plane
            val convexShape = convexA

            val planeNormal = planeShape!!.planeNormal
            val planeConstant = planeShape!!.planeConstant

            val convexWorldTransform = transA
            var convexInPlaneTrans = Transform()
            convexInPlaneTrans = transB.inverse() * convexWorldTransform
            val planeInConvex = convexWorldTransform.inverse() * transB

            val vtx = convexShape.localGetSupportingVertex(planeInConvex.basis * -planeNormal)

            val vtxInPlane = convexInPlaneTrans(vtx)
            val distance = (planeNormal.dot(vtxInPlane) - planeConstant)

            val vtxInPlaneProjected = vtxInPlane - distance * planeNormal
            val vtxInPlaneWorld = transB * vtxInPlaneProjected
            val normalOnSurfaceB = transB.basis * planeNormal

            pointCollector.addContactPoint(normalOnSurfaceB, vtxInPlaneWorld, distance)
        }
    }

    override fun calcTimeOfImpact(fromA: Transform, toA: Transform, fromB: Transform, toB: Transform, result: CastResult): Boolean {

        // compute linear and angular velocity for this interval, to interpolate
        val linVelA = Vec3()
        val angVelA = Vec3()
        val linVelB = Vec3()
        val angVelB = Vec3()
        TransformUtil.calculateVelocity(fromA, toA, 1f, linVelA, angVelA)
        TransformUtil.calculateVelocity(fromB, toB, 1f, linVelB, angVelB)

        val boundingRadiusA = convexA.angularMotionDisc
        val boundingRadiusB = convexB1?.angularMotionDisc ?: 0f

        val maxAngularProjectedVelocity = angVelA.length() * boundingRadiusA + angVelB.length() * boundingRadiusB
        val relLinVel = linVelB - linVelA

        val relLinVelocLength = (linVelB - linVelA).length()

        if ((relLinVelocLength + maxAngularProjectedVelocity) == 0f) return false

        var lambda = 0f

        val n = Vec3(0f)

        var lastLambda = lambda
        var numIter = 0
        // first solution, using GJK
        val radius = 0.001f
//	result.drawCoordSystem(sphereTr);

        val pointCollector1 = PointCollector()

        computeClosestPoints(fromA, fromB, pointCollector1)

        val hasResult = pointCollector1.hasResult
        val c = pointCollector1.pointInWorld

        if (hasResult) {
            var dist = pointCollector1.distance + result.allowedPenetration
            n put pointCollector1.normalOnBInWorld
            var projectedLinearVelocity = relLinVel dot n
            if ((projectedLinearVelocity + maxAngularProjectedVelocity) <= Float.EPSILON) return false

            //not close enough
            while (dist > radius) {
                result.debugDrawer?.drawSphere(c, 0.2f, Vec3(1f))
                var dLambda = 0f
                projectedLinearVelocity = relLinVel dot n
                // don't report time of impact for motion away from the contact normal (or causes minor penetration)
                if ((projectedLinearVelocity + maxAngularProjectedVelocity) <= Float.EPSILON) return false

                dLambda = dist / (projectedLinearVelocity + maxAngularProjectedVelocity)

                lambda += dLambda

                if (lambda > 1f || lambda < 0f) return false
                //todo: next check with relative epsilon
                if (lambda <= lastLambda) return false
                lastLambda = lambda
                //interpolate to next lambda
                val interpolatedTransA = Transform ()
                val interpolatedTransB = Transform ()

                TransformUtil.integrateTransform(fromA, linVelA, angVelA, lambda, interpolatedTransA)
                TransformUtil.integrateTransform(fromB, linVelB, angVelB, lambda, interpolatedTransB)
                val relativeTrans = interpolatedTransB inverseTimes interpolatedTransA

                result.debugDrawer?.drawSphere(interpolatedTransA.origin, 0.2f, Vec3(1f, 0f, 0f))

                result debugDraw lambda

                val pointCollector = PointCollector()
                computeClosestPoints(interpolatedTransA, interpolatedTransB, pointCollector)

                if (pointCollector.hasResult) {
                    dist = pointCollector.distance + result.allowedPenetration
                    c put pointCollector.pointInWorld
                    n put pointCollector.normalOnBInWorld
                } else {
                    result.reportFailure(-1, numIter)
                    return false
                }
                numIter++
                if (numIter > MAX_ITERATIONS) {
                    result.reportFailure(-2, numIter)
                    return false
                }
            }
            result.fraction = lambda
            result.normal put n
            result.hitPoint put c
            return true
        }
        return false
    }
}
