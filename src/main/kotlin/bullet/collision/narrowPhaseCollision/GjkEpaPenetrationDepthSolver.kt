package bullet.collision.narrowPhaseCollision

import bullet.collision.collisionShapes.ConvexShape
import bullet.linearMath.DebugDraw
import bullet.linearMath.Transform
import bullet.linearMath.Vec3

class GjkEpaPenetrationDepthSolver : ConvexPenetrationDepthSolver {

    override fun calcPenDepth(simplexSolver: SimplexSolverInterface,
                              convexA: ConvexShape, convexB: ConvexShape,
                              transA: Transform, transB: Transform,
                              v: Vec3, pa: Vec3, pb: Vec3,
                              debugDraw: DebugDraw?): Boolean {


//	const btScalar				radialmargin(btScalar(0.));

        val guessVector = Vec3(transB.origin - transA.origin)
        val results = GjkEpaSolver2.Results()

        return when {
            GjkEpaSolver2.penetration(convexA, transA,
                    convexB, transB,
                    guessVector, results) -> {
                //	debugDraw->drawLine(results.witnesses[1],results.witnesses[1]+results.normal,btVector3(255,0,0));
                //resultOut->addContactPoint(results.normal,results.witnesses[1],-results.depth);
                pa put results.witnesses[0]
                pb put results.witnesses[1]
                v put results.normal
                true
            }
            GjkEpaSolver2.distance(convexA, transA, convexB, transB, guessVector, results) -> {
                pa put results.witnesses[0]
                pb put results.witnesses[1]
                v put results.normal
                false
            }
            else -> false
        }
    }
}