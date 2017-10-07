package bullet.collision.narrowPhaseCollision

class ConvexShape

/** ConvexPenetrationDepthSolver provides an interface for penetration depth calculation.   */
interface ConvexPenetrationDepthSolver {

    fun calcPenDepth(simplexSolver: SimplexSolverInterface,
                     const btConvexShape* convexA,const btConvexShape* convexB,
    const btTransform& transA,const btTransform& transB,
    btVector3& v, btVector3& pa, btVector3& pb,
    class btIDebugDraw* debugDraw):Boolean
}