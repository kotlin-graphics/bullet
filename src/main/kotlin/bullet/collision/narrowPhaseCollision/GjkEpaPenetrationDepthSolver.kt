package bullet.collision.narrowPhaseCollision

class GjkEpaPenetrationDepthSolver : ConvexPenetrationDepthSolver {

    fun calcPenDepth( btSimplexSolverInterface& simplexSolver,
    const btConvexShape* pConvexA, const btConvexShape* pConvexB,
    const btTransform& transformA, const btTransform& transformB,
    btVector3& v, btVector3& wWitnessOnA, btVector3& wWitnessOnB,
    class btIDebugDraw* debugDraw);
}