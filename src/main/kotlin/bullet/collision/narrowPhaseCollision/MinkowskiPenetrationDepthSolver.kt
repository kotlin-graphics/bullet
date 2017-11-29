package bullet.collision.narrowPhaseCollision

import bullet.DEBUG_DRAW
import bullet.collision.collisionShapes.ConvexShape
import bullet.collision.collisionShapes.MAX_PREFERRED_PENETRATION_DIRECTIONS
import bullet.linearMath.*

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
        minProj += extraSeparation + (convexA.getMarginNonVirtual() + convexB.getMarginNonVirtual())

        val gjkdet = GjkPairDetector(convexA, convexB, simplexSolver, null)

        val offsetDist = minProj
        val offset = minNorm * offsetDist

        val input = DiscreteCollisionDetectorInterface.ClosestPointInput()

        val newOrg = transA.origin + offset

        val displacedTrans = Transform(transA).apply { origin put newOrg }

        input.transformA put displacedTrans
        input.transformB put transB
        input.maximumDistanceSquared = LARGE_FLOAT //minProj;

        val res = IntermediateResult()
        gjkdet.cachedSeparatingAxis put -minNorm
        gjkdet.getClosestPoints(input, res, debugDraw)

        val correctedMinNorm = minProj - res.depth

        //the penetration depth is over-estimated, relax it
        val penetrationRelaxation = 1f
        minNorm *= penetrationRelaxation

        if (res.hasResult) {
            pa put res.pointInWorld - minNorm * correctedMinNorm // TODO check
            pb put res.pointInWorld
            v put minNorm

            if (DEBUG_DRAW)
                debugDraw?.let {
                    val color = Vec3(1, 0, 0)
//                        debugDraw.drawLine(pa, pb, color) TODO
                }
        }
        return res.hasResult
    }

    companion object {
        val penetrationDirections = arrayOf(
                Vec3(+0.000000, -0.000000, -1.000000),
                Vec3(+0.723608, -0.525725, -0.447219),
                Vec3(-0.276388, -0.850649, -0.447219),
                Vec3(-0.894426, -0.000000, -0.447216),
                Vec3(-0.276388, +0.850649, -0.447220),
                Vec3(+0.723608, +0.525725, -0.447219),
                Vec3(+0.276388, -0.850649, +0.447220),
                Vec3(-0.723608, -0.525725, +0.447219),
                Vec3(-0.723608, +0.525725, +0.447219),
                Vec3(+0.276388, +0.850649, +0.447219),
                Vec3(+0.894426, +0.000000, +0.447216),
                Vec3(-0.000000, +0.000000, +1.000000),
                Vec3(+0.425323, -0.309011, -0.850654),
                Vec3(-0.162456, -0.499995, -0.850654),
                Vec3(+0.262869, -0.809012, -0.525738),
                Vec3(+0.425323, +0.309011, -0.850654),
                Vec3(+0.850648, -0.000000, -0.525736),
                Vec3(-0.525730, -0.000000, -0.850652),
                Vec3(-0.688190, -0.499997, -0.525736),
                Vec3(-0.162456, +0.499995, -0.850654),
                Vec3(-0.688190, +0.499997, -0.525736),
                Vec3(+0.262869, +0.809012, -0.525738),
                Vec3(+0.951058, +0.309013, +0.000000),
                Vec3(+0.951058, -0.309013, +0.000000),
                Vec3(+0.587786, -0.809017, +0.000000),
                Vec3(+0.000000, -1.000000, +0.000000),
                Vec3(-0.587786, -0.809017, +0.000000),
                Vec3(-0.951058, -0.309013, -0.000000),
                Vec3(-0.951058, +0.309013, -0.000000),
                Vec3(-0.587786, +0.809017, -0.000000),
                Vec3(-0.000000, +1.000000, -0.000000),
                Vec3(+0.587786, +0.809017, -0.000000),
                Vec3(+0.688190, -0.499997, +0.525736),
                Vec3(-0.262869, -0.809012, +0.525738),
                Vec3(-0.850648, +0.000000, +0.525736),
                Vec3(-0.262869, +0.809012, +0.525738),
                Vec3(+0.688190, +0.499997, +0.525736),
                Vec3(+0.525730, +0.000000, +0.850652),
                Vec3(+0.162456, -0.499995, +0.850654),
                Vec3(-0.425323, -0.309011, +0.850654),
                Vec3(-0.425323, +0.309011, +0.850654),
                Vec3(+0.162456, +0.499995, +0.850654))
    }
}