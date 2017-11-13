package bullet.collision.narrowPhaseCollision

import bullet.collision.collisionShapes.ConvexShape
import bullet.collision.collisionShapes.MAX_PREFERRED_PENETRATION_DIRECTIONS
import bullet.linearMath.DebugDraw
import bullet.linearMath.LARGE_FLOAT
import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import bullet.linearMath.times

private val NUM_UNITSPHERE_POINTS = 42

class MinkowskiPenetrationDepthSolver : ConvexPenetrationDepthSolver {

    override fun calcPenDepth(simplexSolver: SimplexSolverInterface,
                              convexA: ConvexShape, convexB: ConvexShape,
                              transA: Transform, transB: Transform,
                              v: Vec3, pa: Vec3, pb: Vec3,
                              debugDraw: DebugDraw?): Boolean {

        val check2d = convexA.isConvex2d && convexB.isConvex2d

        class IntermediateResult : DiscreteCollisionDetectorInterface.Result {

            val normalOnBInWorld = Vec3()
            val pointInWorld = Vec3()
            var depth = 0f
            var hasResult = false

            override fun setShapeIdentifiersA(partId0: Int, index0: Int) = Unit
            override fun setShapeIdentifiersB(partId1: Int, index1: Int) = Unit
            override fun addContactPoint(normalOnBInWorld: Vec3, pointInWorld: Vec3, depth: Float) {
                this.normalOnBInWorld put normalOnBInWorld
                this.pointInWorld put pointInWorld
                this.depth = depth
                hasResult = true
            }
        }

        // just take fixed number of orientation, and sample the penetration depth in that direction
        var minProj = LARGE_FLOAT
        val minNorm = Vec3()
        val minA = Vec3()
        val minB = Vec3()
        val seperatingAxisInA = Vec3()
        val seperatingAxisInB = Vec3()
        val pInA = Vec3()
        val qInB = Vec3()
        val pWorld = Vec3()
        val qWorld = Vec3()
        val w = Vec3()

        val supportVerticesABatch = Array(NUM_UNITSPHERE_POINTS + MAX_PREFERRED_PENETRATION_DIRECTIONS * 2, { Vec3() })
        val supportVerticesBBatch = Array(NUM_UNITSPHERE_POINTS + MAX_PREFERRED_PENETRATION_DIRECTIONS * 2, { Vec3() })
        val seperatingAxisInABatch = Array(NUM_UNITSPHERE_POINTS + MAX_PREFERRED_PENETRATION_DIRECTIONS * 2, { Vec3() })
        val seperatingAxisInBBatch = Array(NUM_UNITSPHERE_POINTS + MAX_PREFERRED_PENETRATION_DIRECTIONS * 2, { Vec3() })

        var numSampleDirections = NUM_UNITSPHERE_POINTS

        for (i in 0 until numSampleDirections) {
            val norm = penetrationDirections[i]
            seperatingAxisInABatch[i] = -norm * transA.basis
            seperatingAxisInBBatch[i] = norm * transB.basis
        }

        val numPDA = convexB.numPreferredPenetrationDirections
        if (numPDA != 0)
            for (i in 0 until numPDA) {
                val norm = Vec3()
                convexA.getPreferredPenetrationDirection(i, norm)
                norm put transA.basis * norm
                penetrationDirections[numSampleDirections] = norm
                seperatingAxisInABatch[numSampleDirections] = -norm * transA.basis
                seperatingAxisInBBatch[numSampleDirections] = norm * transB.basis
                numSampleDirections++
            }

        val numPDB = convexB.numPreferredPenetrationDirections
        if (numPDB != 0)
            for (i in 0 until numPDB) {
                val norm = Vec3()
                convexB.getPreferredPenetrationDirection(i, norm)
                norm put transB.basis * norm    // TODO check order
                penetrationDirections[numSampleDirections] = norm
                seperatingAxisInABatch[numSampleDirections] = -norm * transA.basis
                seperatingAxisInBBatch[numSampleDirections] = norm * transB.basis
                numSampleDirections++
            }

        convexA.batchedUnitVectorGetSupportingVertexWithoutMargin(seperatingAxisInABatch, supportVerticesABatch, numSampleDirections)
        convexB.batchedUnitVectorGetSupportingVertexWithoutMargin(seperatingAxisInBBatch, supportVerticesBBatch, numSampleDirections)

        for (i in 0 until numSampleDirections) {
            val norm = penetrationDirections[i]
            if (check2d) norm[2] = 0f
            if (norm.length2() > 0.01f) {

                seperatingAxisInA put seperatingAxisInABatch[i]
                seperatingAxisInB put seperatingAxisInBBatch[i]

                pInA put supportVerticesABatch[i]
                qInB put supportVerticesBBatch[i]

                pWorld put transA(pInA)
                qWorld put transB(qInB)
                if (check2d) {
                    pWorld[2] = 0f
                    qWorld[2] = 0f
                }

                w put qWorld - pWorld   // TODO order
                val delta = norm dot w
                //find smallest delta
                if (delta < minProj) {
                    minProj = delta
                    minNorm put norm
                    minA put pWorld
                    minB put qWorld
                }
            }
        }

        //add the margins
        minA += minNorm * convexA.getMarginNonVirtual()
        minB -= minNorm * convexB.getMarginNonVirtual()
        //no penetration
        if (minProj < 0f) return false

        val extraSeparation = 0.5f // scale dependent
        minProj += extraSeparation + (convexA.getMarginNonVirtual()+convexB.getMarginNonVirtual())

        val gjkdet = GjkPairDetector (convexA, convexB, simplexSolver, null)

        val offsetDist = minProj
        val offset = minNorm * offsetDist

        val input = GjkPairDetector.ClosestPointInput

        btVector3 newOrg = transA . getOrigin () + offset;

        btTransform displacedTrans = transA;
        displacedTrans.setOrigin(newOrg);

        input.m_transformA = displacedTrans;
        input.m_transformB = transB;
        input.m_maximumDistanceSquared = btScalar(BT_LARGE_FLOAT);//minProj;

        btIntermediateResult res;
        gjkdet.setCachedSeperatingAxis(-minNorm);
        gjkdet.getClosestPoints(input, res, debugDraw);

        btScalar correctedMinNorm = minProj -res.m_depth;


        //the penetration depth is over-estimated, relax it
        btScalar penetration_relaxation = btScalar (1.);
        minNorm *= penetration_relaxation;


        if (res.m_hasResult) {

            pa = res.m_pointInWorld - minNorm * correctedMinNorm;
            pb = res.m_pointInWorld;
            v = minNorm;

            #ifdef DEBUG_DRAW
                    if (debugDraw) {
                        btVector3 color (1, 0, 0);
                        debugDraw->drawLine(pa, pb, color);
                    }
            #endif//DEBUG_DRAW


        }
        return res.m_hasResult;
    }

}