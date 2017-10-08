package bullet.collision.narrowPhaseCollision

import bullet.linearMath.DebugDraw
import bullet.linearMath.Transform
import bullet.linearMath.Vec3

class GjkEpaPenetrationDepthSolver : ConvexPenetrationDepthSolver {

    fun calcPenDepth(simplexSolver: SimplexSolverInterface,
                     convexA: ConvexShape, convexB: ConvexShape,
                     transformA: Transform, transformB: Transform,
                     v: Vec3, witnessOnA: Vec3, witnessOnB: Vec3,
                     debugDraw: DebugDraw): Boolean {


//	const btScalar				radialmargin(btScalar(0.));

        val guessVector = Vec3(transformB.origin - transformA.origin)
        val results = GjkEpaSolver2.Results()

        return when {
            GjkEpaSolver2.penetration(convexA, transformA,
                    convexB, transformB,
                    guessVector, results) -> {
                //	debugDraw->drawLine(results.witnesses[1],results.witnesses[1]+results.normal,btVector3(255,0,0));
                //resultOut->addContactPoint(results.normal,results.witnesses[1],-results.depth);
                witnessOnA put results.witnesses[0]
                witnessOnB put results.witnesses[1]
                v put results.normal
                true
            }
            GjkEpaSolver2.distance(convexA, transformA, convexB, transformB, guessVector, results) -> {
                witnessOnA put results.witnesses[0]
                witnessOnB put results.witnesses[1]
                v put results.normal
                false
            }
            else -> false
        }
    }
}