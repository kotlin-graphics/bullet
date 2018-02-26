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

import bullet.DEBUG
import bullet.EPSILON
import bullet.collision.collisionShapes.ConvexShape
import bullet.linearMath.DebugDraw
import bullet.linearMath.LARGE_FLOAT
import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import bullet.linearMath.times
import kotlin.math.sqrt

//temp globals, to improve GJK/EPA/penetration calculations
private var numDeepPenetrationChecks = 0
private var numGjkChecks = 0

private val REL_ERROR2 = 1.0e-6f
private val gjkEpaPenetrationTolerance = 0.001f

/** GjkPairDetector uses GJK to implement the DiscreteCollisionDetector    */
class GjkPairDetector(
        var minkowskiA: ConvexShape, var minkowskiB: ConvexShape,
        val shapeTypeA: Int, val shapeTypeB: Int,
        val marginA: Float, val marginB: Float,
        val simplexSolver: SimplexSolverInterface?,
        val penetrationDepthSolver: ConvexPenetrationDepthSolver?
) : DiscreteCollisionDetectorInterface {

    constructor(objectA: ConvexShape, objectB: ConvexShape, simplexSolver: SimplexSolverInterface, penetrationDepthSolver: ConvexPenetrationDepthSolver?) :
            this(objectA, objectB, objectA.shapeType.i, objectB.shapeType.i, objectA.margin, objectB.margin, simplexSolver, penetrationDepthSolver)

    val cachedSeparatingAxis = Vec3(0f, 1f, 0f)
    var ignoreMargin = false
    var cachedSeparatingDistance = 0f

    //some debugging to fix degeneracy problems
    var lastUsedMethod = -1
    var curIter = 0
    var degenerateSimplex = 0
    var catchDegeneracies = 1
    var fixContactNormalDirection = 1

    override fun getClosestPoints(input: DiscreteCollisionDetectorInterface.ClosestPointInput, output: DiscreteCollisionDetectorInterface.Result,
                         debugDraw: DebugDraw?, swapResults: Boolean) = getClosestPointsNonVirtual(input, output, debugDraw)

    fun getClosestPointsNonVirtual(input: DiscreteCollisionDetectorInterface.ClosestPointInput, output: DiscreteCollisionDetectorInterface.Result,
                                   debugDraw: DebugDraw?) {

        cachedSeparatingDistance = 0f

        var distance = 0f
        val normalInB = Vec3()

        val pointOnA = Vec3()
        val pointOnB = Vec3()
        val localTransA = Transform(input.transformA)
        val localTransB = Transform(input.transformB)
        val positionOffset = (localTransA.origin + localTransB.origin) * 0.5f
        localTransA.origin minusAssign positionOffset
        localTransB.origin minusAssign positionOffset

        val check2d = minkowskiA.isConvex2d && minkowskiB.isConvex2d

        var marginA = marginA
        var marginB = marginB

        numGjkChecks++

        //for CCD we don't use margins
        if (ignoreMargin) {
            marginA = 0f
            marginB = 0f
        }

        curIter = 0
        val gjkMaxIter = 1000   // this is to catch invalid input, perhaps check for #NaN?
        cachedSeparatingAxis.put(0f, 1f, 0f)

        var isValid = false
        var checkSimplex = false
        val checkPenetration = true
        degenerateSimplex = 0

        lastUsedMethod = -1

        var squaredDistance = LARGE_FLOAT
        var delta = 0f

        val margin = marginA + marginB

        simplexSolver!!.reset()

        while (true) {
            val seperatingAxisInA = (-cachedSeparatingAxis) * input.transformA.basis
            val seperatingAxisInB = cachedSeparatingAxis * input.transformB.basis

            val pInA = minkowskiA localGetSupportVertexWithoutMarginNonVirtual seperatingAxisInA
            val qInB = minkowskiB localGetSupportVertexWithoutMarginNonVirtual seperatingAxisInB

            val pWorld = localTransA(pInA)
            val qWorld = localTransB(qInB)

            if (check2d) {
                pWorld[2] = 0f
                qWorld[2] = 0f
            }

            val w = pWorld - qWorld
            delta = cachedSeparatingAxis dot w

            // potential exit, they don't overlap
            if (delta > 0f && delta * delta > squaredDistance * input.maximumDistanceSquared) {
                degenerateSimplex = 10
                checkSimplex = true
                //checkPenetration = false;
                break
            }

            // exit 0: the new point is already in the simplex, or we didn't come any closer
            if (simplexSolver inSimplex w) {
                degenerateSimplex = 1
                checkSimplex = true
                break
            }
            // are we getting any closer ?
            val f0 = squaredDistance - delta
            val f1 = squaredDistance * REL_ERROR2

            if (f0 <= f1) {
                degenerateSimplex = if (f0 <= 0f) 2 else 11
                checkSimplex = true
                break
            }

            // add current vertex to simplex
            simplexSolver.addVertex(w, pWorld, qWorld)
            val newCachedSeparatingAxis = Vec3()

            // calculate the closest point to the origin (update vector v)
            if (!simplexSolver.closest(newCachedSeparatingAxis)) {
                degenerateSimplex = 3
                checkSimplex = true
                break
            }

            if (newCachedSeparatingAxis.length2() < REL_ERROR2) {
                cachedSeparatingAxis put newCachedSeparatingAxis
                degenerateSimplex = 6
                checkSimplex = true
                break
            }

            val previousSquaredDistance = squaredDistance
            squaredDistance = newCachedSeparatingAxis.length2()

            // redundant m_simplexSolver->compute_points(pointOnA, pointOnB);

            // are we getting any closer ?
            if (previousSquaredDistance - squaredDistance <= Float.EPSILON * previousSquaredDistance) {
//				m_simplexSolver->backup_closest(m_cachedSeparatingAxis);
                checkSimplex = true
                degenerateSimplex = 12
                break
            }

            cachedSeparatingAxis put newCachedSeparatingAxis

            // degeneracy, this is typically due to invalid/uninitialized worldtransforms for a btCollisionObject
            if (curIter++ > gjkMaxIter) {
                if (DEBUG) {
//                    printf("btGjkPairDetector maxIter exceeded:%i\n", m_curIter)
//                    printf("sepAxis=(%f,%f,%f), squaredDistance = %f, shapeTypeA=%i,shapeTypeB=%i\n",
//                            m_cachedSeparatingAxis.getX(),
//                            m_cachedSeparatingAxis.getY(),
//                            m_cachedSeparatingAxis.getZ(),
//                            squaredDistance,
//                            m_minkowskiA->getShapeType(),
//                    m_minkowskiB->getShapeType())
                }
                break
            }

            val check = !simplexSolver.fullSimplex
            //bool check = (!m_simplexSolver->fullSimplex() && squaredDistance > SIMD_EPSILON * m_simplexSolver->maxVertex());

            if (!check) {
                //do we need this backup_closest here ?
//				m_simplexSolver->backup_closest(m_cachedSeparatingAxis);
                degenerateSimplex = 13
                break
            }
        }

        if (checkSimplex) {
            simplexSolver.computePoints(pointOnA, pointOnB)
            normalInB put cachedSeparatingAxis

            val lenSqr = cachedSeparatingAxis.length2()

            //valid normal
            if (lenSqr < REL_ERROR2) degenerateSimplex = 5
            if (lenSqr > Float.EPSILON * Float.EPSILON) {
                val rlen = 1f / sqrt(lenSqr)
                normalInB *= rlen //normalize

                val s = sqrt(squaredDistance)

                assert(s > 0f)
                pointOnA -= cachedSeparatingAxis * (marginA / s)
                pointOnB += cachedSeparatingAxis * (marginB / s)
                distance = (1f / rlen) - margin
                isValid = true
                lastUsedMethod = 1
            } else lastUsedMethod = 2
        }

        val catchDegeneratePenetrationCase = catchDegeneracies != 0 && penetrationDepthSolver != null &&
                degenerateSimplex != 0 && (distance + margin) < gjkEpaPenetrationTolerance

        //if (checkPenetration && !isValid)
        if (checkPenetration && (!isValid || catchDegeneratePenetrationCase)) {
            // penetration case
            // if there is no way to handle penetrations, bail out
            if (penetrationDepthSolver != null) {
                // Penetration depth case.
                val tmpPointOnA = Vec3()
                val tmpPointOnB = Vec3()

                numDeepPenetrationChecks++
                cachedSeparatingAxis put 0f

                val isValid2 = penetrationDepthSolver.calcPenDepth(simplexSolver, minkowskiA, minkowskiB,
                        localTransA, localTransB, cachedSeparatingAxis, tmpPointOnA, tmpPointOnB, debugDraw)

                if (isValid2) {
                    val tmpNormalInB = tmpPointOnB - tmpPointOnA
                    var lenSqr = tmpNormalInB.length2()
                    if (lenSqr <= Float.EPSILON * Float.EPSILON) {
                        tmpNormalInB put cachedSeparatingAxis
                        lenSqr = cachedSeparatingAxis.length2()
                    }

                    if (lenSqr > Float.EPSILON * Float.EPSILON) {
                        tmpNormalInB /= sqrt(lenSqr)
                        val distance2 = -(tmpPointOnA - tmpPointOnB).length()
                        lastUsedMethod = 3
                        //only replace valid penetrations when the result is deeper (check)
                        if (!isValid || (distance2 < distance)) {
                            distance = distance2
                            pointOnA put tmpPointOnA
                            pointOnB put tmpPointOnB
                            normalInB put tmpNormalInB

                            isValid = true
                        } else lastUsedMethod = 8
                    } else lastUsedMethod = 9
                } else {
                    /*  this is another degenerate case, where the initial GJK calculation reports a degenerate case
                        EPA reports no penetration, and the second GJK (using the supporting vector without margin)
                        reports a valid positive distance. Use the results of the second GJK instead of failing.
                        thanks to Jacob.Langford for the reproduction case
                        http://code.google.com/p/bullet/issues/detail?id=250    */
                    if (cachedSeparatingAxis.length2() > 0f) {
                        val distance2 = (tmpPointOnA - tmpPointOnB).length() - margin
                        //only replace valid distances when the distance is less
                        if (!isValid || distance2 < distance) {
                            distance = distance2
                            pointOnA put tmpPointOnA
                            pointOnB put tmpPointOnB
                            pointOnA -= cachedSeparatingAxis * marginA
                            pointOnB += cachedSeparatingAxis * marginB
                            normalInB put cachedSeparatingAxis
                            normalInB.normalize()

                            isValid = true
                            lastUsedMethod = 6
                        } else lastUsedMethod = 5
                    }
                }
            }
        }
        if (isValid && (distance < 0 || distance * distance < input.maximumDistanceSquared)) {

            cachedSeparatingAxis put normalInB
            cachedSeparatingDistance = distance

            /*  todo: need to track down this EPA penetration solver degeneracy
                the penetration solver reports penetration but the contact normal connecting the contact points
                is pointing in the opposite direction, until then, detect the issue and revert the normal   */
            val d1 = run {
                val seperatingAxisInA = normalInB * input.transformA.basis
                val seperatingAxisInB = -normalInB * input.transformB.basis

                val pInA = minkowskiA localGetSupportVertexWithoutMarginNonVirtual seperatingAxisInA
                val qInB = minkowskiB localGetSupportVertexWithoutMarginNonVirtual seperatingAxisInB

                val pWorld = localTransA(pInA)
                val qWorld = localTransB(qInB)
                val w = pWorld - qWorld
                -normalInB dot w
            }
            val d0 = run {
                val seperatingAxisInA = (-normalInB) * input.transformA.basis
                val seperatingAxisInB = normalInB * input.transformB.basis

                val pInA = minkowskiA localGetSupportVertexWithoutMarginNonVirtual seperatingAxisInA
                val qInB = minkowskiB localGetSupportVertexWithoutMarginNonVirtual seperatingAxisInB

                val pWorld = localTransA(pInA)
                val qWorld = localTransB(qInB)
                val w = pWorld - qWorld
                normalInB dot w
            }
            if (d1 > d0) {
                lastUsedMethod = 10
                normalInB *= -1f
            }
            output.addContactPoint(normalInB, pointOnB + positionOffset, distance)
        }
    }
}