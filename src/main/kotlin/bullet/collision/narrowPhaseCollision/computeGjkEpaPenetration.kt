package bullet.collision.narrowPhaseCollision

import bullet.ConvexTemplate
import bullet.DEBUG
import bullet.DistanceTemplate
import bullet.EPSILON
import bullet.linearMath.LARGE_FLOAT
import bullet.linearMath.Vec3
import bullet.linearMath.times
import kotlin.math.sqrt

fun gjkEpaCalcPenDepth(a: ConvexTemplate, b: ConvexTemplate,
                       colDesc: GjkCollisionDescription,
                       v: Vec3, witnessOnA: Vec3, witnessOnB: Vec3): Boolean {

    //	const btScalar				radialmargin(btScalar(0.));

    val guessVector = Vec3(b.worldTrans.origin - a.worldTrans.origin)//?? why not use the GJK input?

    val results = GjkEpaSolver3.Results()

    if (GjkEpaSolver3.penetration(a, b, guessVector, results)) {
        //	debugDraw->drawLine(results.witnesses[1],results.witnesses[1]+results.normal,btVector3(255,0,0));
        //resultOut->addContactPoint(results.normal,results.witnesses[1],-results.depth);
        witnessOnA put results.witnesses[0]
        witnessOnB put results.witnesses[1]
        v put results.normal
        return true
    }
    if (GjkEpaSolver3.distance(a, b, guessVector, results)) {
        witnessOnA put results.witnesses[0]
        witnessOnB put results.witnesses[1]
        v put results.normal
    }
    return false
}

fun computeGjkEpaPenetration(a: ConvexTemplate, b: ConvexTemplate, colDesc: GjkCollisionDescription,
                             simplexSolver: VoronoiSimplexSolver, distInfo: DistanceTemplate): Int {

    val catchDegeneracies = true
    var cachedSeparatingDistance = 0f

    var distance = 0f
    val normalInB = Vec3()

    val pointOnA = Vec3()
    val pointOnB = Vec3()
    val localTransA = a.worldTrans
    val localTransB = b.worldTrans

    val marginA = a.margin
    val marginB = b.margin

    var curIter = 0
    val gjkMaxIter = colDesc.maxGjkIterations   //this is to catch invalid input, perhaps check for #NaN?
    var cachedSeparatingAxis = colDesc.firstDir

    var isValid = false
    var checkSimplex = false
    val checkPenetration = true
    var degenerateSimplex = 0

    var lastUsedMethod = -1

    run {
        var squaredDistance = LARGE_FLOAT

        val margin = marginA + marginB

        simplexSolver.reset()

        while (true) {

            val seperatingAxisInA = -cachedSeparatingAxis * localTransA.basis
            val seperatingAxisInB = cachedSeparatingAxis * localTransB.basis

            val pInA = a.getLocalSupportWithoutMargin(seperatingAxisInA)
            val qInB = b.getLocalSupportWithoutMargin(seperatingAxisInB)

            val pWorld = localTransA * pInA
            val qWorld = localTransB * qInB

            val w = pWorld - qWorld
            val delta = cachedSeparatingAxis dot w

            // potential exit, they don't overlap
            if (delta > 0f && delta * delta > squaredDistance * colDesc.maximumDistanceSquared) {
                degenerateSimplex = 10
                checkSimplex = true
                //checkPenetration = false;
                break
            }

            //exit 0: the new point is already in the simplex, or we didn't come any closer
            if (simplexSolver.inSimplex(w)) {
                degenerateSimplex = 1
                checkSimplex = true
                break
            }
            // are we getting any closer ?
            val f0 = squaredDistance - delta
            val f1 = squaredDistance * colDesc.gjkRelError2

            if (f0 <= f1) {
                degenerateSimplex = if (f0 <= 0f) 2 else 11
                checkSimplex = true
                break
            }

            //add current vertex to simplex
            simplexSolver.addVertex(w, pWorld, qWorld)
            val newCachedSeparatingAxis = Vec3()

            //calculate the closest point to the origin (update vector v)
            if (!simplexSolver.closest(newCachedSeparatingAxis)) {
                degenerateSimplex = 3
                checkSimplex = true
                break
            }

            if (newCachedSeparatingAxis.length2() < colDesc.gjkRelError2) {
                cachedSeparatingAxis = newCachedSeparatingAxis
                degenerateSimplex = 6
                checkSimplex = true
                break
            }

            val previousSquaredDistance = squaredDistance
            squaredDistance = newCachedSeparatingAxis.length2()

            //redundant m_simplexSolver->compute_points(pointOnA, pointOnB);

            //are we getting any closer ?
            if (previousSquaredDistance - squaredDistance <= Float.EPSILON * previousSquaredDistance) {
                //				m_simplexSolver->backup_closest(m_cachedSeparatingAxis);
                checkSimplex = true
                degenerateSimplex = 12
                break
            }

            cachedSeparatingAxis = newCachedSeparatingAxis

            //degeneracy, this is typically due to invalid/uninitialized worldtransforms for a btCollisionObject
            if (curIter++ > gjkMaxIter) {
                if (DEBUG) {
                    println("btGjkPairDetector maxIter exceeded: $curIter")
                    println("sepAxis $cachedSeparatingAxis, squaredDistance = $squaredDistance")
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
            if (lenSqr < 0.0001) degenerateSimplex = 5
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
            } else
                lastUsedMethod = 2
        }

        val catchDegeneratePenetrationCase = catchDegeneracies && degenerateSimplex != 0 && (distance + margin) < 0.01f

        //if (checkPenetration && !isValid)
        if (checkPenetration && (!isValid || catchDegeneratePenetrationCase)) {
            //penetration case

            //if there is no way to handle penetrations, bail out

            // Penetration depth case.
            val tmpPointOnA = Vec3()
            val tmpPointOnB = Vec3()

            cachedSeparatingAxis put 0f

            val isValid2 = gjkEpaCalcPenDepth(a, b, colDesc, cachedSeparatingAxis, tmpPointOnA, tmpPointOnB)

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
                    //only replace valid penetrations when the result is deeper (check)
                    if (!isValid || distance2 < distance) {
                        distance = distance2
                        pointOnA put tmpPointOnA
                        pointOnB put tmpPointOnB
                        normalInB put tmpNormalInB

                        isValid = true
                        lastUsedMethod = 3
                    } else
                        lastUsedMethod = 8
                } else
                    lastUsedMethod = 9
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
                    } else
                        lastUsedMethod = 5
                }
            }
        }
    }

    if (isValid && ((distance < 0) || (distance * distance < colDesc.maximumDistanceSquared))) {

        cachedSeparatingAxis = normalInB
        cachedSeparatingDistance = distance
        distInfo.distance = distance
        distInfo.normalBtoA = normalInB
        distInfo.pointOnB = pointOnB
        distInfo.pointOnA = pointOnB + normalInB * distance
        return 0
    }
    return -lastUsedMethod
}