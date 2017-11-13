package bullet.collision.narrowPhaseCollision

import bullet.collision.collisionShapes.ConvexShape
import bullet.linearMath.DebugDraw
import bullet.linearMath.Transform
import bullet.linearMath.Vec3

/** ConvexPenetrationDepthSolver provides an interface for penetration depth calculation.   */
interface ConvexPenetrationDepthSolver {

    fun calcPenDepth(simplexSolver: SimplexSolverInterface,
                     convexA: ConvexShape, convexB: ConvexShape,
                     transA: Transform, transB: Transform,
                     v: Vec3, pa: Vec3, pb: Vec3,
                     debugDraw: DebugDraw?): Boolean
}