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

package bullet.collision.collisionDispatch

import bullet.*
import bullet.collision.broadphaseCollision.CollisionAlgorithmConstructionInfo
import bullet.collision.broadphaseCollision.DispatcherInfo
import bullet.collision.collisionShapes.*
import bullet.collision.narrowPhaseCollision.*
import bullet.linearMath.*
import bullet.collision.broadphaseCollision.BroadphaseNativeTypes as Bnt

/* Enabling USE_SEPDISTANCE_UTIL2 requires 100% reliable distance computation. However, when using large size ratios
    GJK can be imprecise so the distance is not conservative. In that case, enabling this USE_SEPDISTANCE_UTIL2 would
    result in failing/missing collisions.
    Either improve GJK for large size ratios (testing a 100 units versus a 0.1 unit object) or only enable the util
    for certain pairs that have a small size ratio  */

/** The convexConvexAlgorithm collision algorithm implements time of impact, convex closest points and penetration depth
 *  calculations between two convex objects.
 *  Multiple contact points are calculated by perturbing the orientation of the smallest object orthogonal to the
 *  separating normal.
 *  This idea was described by Gino van den Bergen in this forum topic
 *  http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=4&t=288&p=888#p888 */
class ConvexConvexAlgorithm(var manifold: PersistentManifold?, ci: CollisionAlgorithmConstructionInfo,
                            body0Wrap: CollisionObjectWrapper, body1Wrap: CollisionObjectWrapper,
                            val solver: ConvexPenetrationDepthSolver, val numPerturbationIterations: Int,
                            val minimumPointsPerturbationThreshold: Int) : ActivatingCollisionAlgorithm(ci, body0Wrap, body1Wrap) {

    val sepDistance = ConvexSeparatingDistanceUtil(
            (body0Wrap.collisionShape as ConvexShape).angularMotionDisc,
            (body1Wrap.collisionShape as ConvexShape).angularMotionDisc).takeIf { USE_SEPDISTANCE_UTIL2 }

    val worldVertsB1 = ArrayList<Vec3>()
    val worldVertsB2 = ArrayList<Vec3>()

    var ownManifold = false
    var lowLevelOfDetail = false

    var disableCcd = false

    /** Convex-Convex collision algorithm   */
    override fun processCollision(body0Wrap: CollisionObjectWrapper, body1Wrap: CollisionObjectWrapper, dispatchInfo: DispatcherInfo,
                                  resultOut: ManifoldResult) {

        val body0 = body0Wrap
        val body1 = body1Wrap
        if (manifold == null) {
            //swapped?
            manifold = dispatcher!!.getNewManifold(body0.collisionObject!!, body1.collisionObject!!)
            ownManifold = true
        }
        resultOut.manifold = manifold

        //comment-out next line to test multi-contact generation
        //resultOut->getPersistentManifold()->clearManifold();

        val min0 = body0.collisionShape as ConvexShape
        val min1 = body1.collisionShape as ConvexShape

        val normalOnB = Vec3()
        val pointOnBWorld = Vec3()
        if (!DISABLE_CAPSULE_CAPSULE_COLLIDER) {
            if (min0.shapeType == Bnt.CAPSULE_SHAPE_PROXYTYPE && min1.shapeType == Bnt.CAPSULE_SHAPE_PROXYTYPE) {
                //m_manifoldPtr->clearManifold();
                val capsuleA = min0 as CapsuleShape
                val capsuleB = min1 as CapsuleShape

                val threshold = manifold!!.contactBreakingThreshold

                val dist = capsuleCapsuleDistance(normalOnB, pointOnBWorld, capsuleA.halfHeight, capsuleA.radius,
                        capsuleB.halfHeight, capsuleB.radius, capsuleA.upAxis, capsuleB.upAxis,
                        body0.worldTransform, body1.worldTransform, threshold)

                if (dist < threshold) {
                    assert(normalOnB.length2() >= Float.EPSILON * Float.EPSILON)
                    resultOut.addContactPoint(normalOnB, pointOnBWorld, dist)
                }
                resultOut.refreshContactPoints()
                return
            }

            if (min0.shapeType == Bnt.CAPSULE_SHAPE_PROXYTYPE && min1.shapeType == Bnt.SPHERE_SHAPE_PROXYTYPE) {
                //m_manifoldPtr->clearManifold();
                val capsuleA = min0 as CapsuleShape
                val capsuleB = min1 as SphereShape

                val threshold = manifold!!.contactBreakingThreshold

                val dist = capsuleCapsuleDistance(normalOnB, pointOnBWorld, capsuleA.halfHeight, capsuleA.radius, 0f,
                        capsuleB.radius, capsuleA.upAxis, 1, body0.worldTransform, body1.worldTransform, threshold)

                if (dist < threshold) {
                    assert(normalOnB.length2() >= Float.EPSILON * Float.EPSILON)
                    resultOut.addContactPoint(normalOnB, pointOnBWorld, dist)
                }
                resultOut.refreshContactPoints()
                return
            }

            if (min0.shapeType == Bnt.SPHERE_SHAPE_PROXYTYPE && min1.shapeType == Bnt.CAPSULE_SHAPE_PROXYTYPE) {
                //m_manifoldPtr->clearManifold();
                val capsuleA = min0 as SphereShape
                val capsuleB = min1 as CapsuleShape

                val threshold = manifold!!.contactBreakingThreshold

                val dist = capsuleCapsuleDistance(normalOnB, pointOnBWorld, 0f, capsuleA.radius, capsuleB.halfHeight,
                        capsuleB.radius, 1, capsuleB.upAxis, body0.worldTransform, body1.worldTransform, threshold)

                if (dist < threshold) {
                    assert(normalOnB.length2() >= Float.EPSILON * Float.EPSILON)
                    resultOut.addContactPoint(normalOnB, pointOnBWorld, dist)
                }
                resultOut.refreshContactPoints()
                return
            }
        }

        if (USE_SEPDISTANCE_UTIL2)
            if(dispatchInfo.useConvexConservativeDistanceUtil)
                sepDistance!!.updateSeparatingDistance(body0.worldTransform, body1.worldTransform)

        val cond = when(USE_SEPDISTANCE_UTIL2) {
            true -> !dispatchInfo.useConvexConservativeDistanceUtil || sepDistance!!.separatingDistance <= 0f
            else -> true
        }
        if (cond) {

            val input = DiscreteCollisionDetectorInterface.ClosestPointInput()
            val simplexSolver = VoronoiSimplexSolver()
            val gjkPairDetector = GjkPairDetector(min0, min1, simplexSolver, solver)
            //TODO: if (dispatchInfo.m_useContinuous)
            gjkPairDetector.minkowskiA = min0
            gjkPairDetector.minkowskiB = min1

            var `else` = !USE_SEPDISTANCE_UTIL2
            if (USE_SEPDISTANCE_UTIL2)
                if(dispatchInfo.useConvexConservativeDistanceUtil)
                    input.maximumDistanceSquared = LARGE_FLOAT
                else `else` = true
            if (`else`) {
                input.maximumDistanceSquared = min0.margin + min1.margin + manifold!!.contactBreakingThreshold + resultOut.closestPointDistanceThreshold
                input.maximumDistanceSquared *= input.maximumDistanceSquared
            }
            input.transformA put body0.worldTransform
            input.transformB put body1.worldTransform

            var sepDist = 0f
            if (USE_SEPDISTANCE_UTIL2 && dispatchInfo.useConvexConservativeDistanceUtil) {
                sepDist = gjkPairDetector.cachedSeparatingDistance
                if (sepDist > Float.EPSILON)
                    sepDist += dispatchInfo.convexConservativeDistanceThreshold
                //now perturbe directions to get multiple contact points
            }

            if (min0.isPolyhedral && min1.isPolyhedral) {

                class DummyResult : DiscreteCollisionDetectorInterface.Result {
                    override fun setShapeIdentifiersA(partId0: Int, index0: Int) = Unit
                    override fun setShapeIdentifiersB(partId1: Int, index1: Int) = Unit
                    override fun addContactPoint(normalOnBInWorld: Vec3, pointInWorld: Vec3, depth: Float) = Unit
                }

                class WithoutMarginResult(val originalResult: DiscreteCollisionDetectorInterface.Result, val marginOnA: Float,
                                          val marginOnB: Float) : DiscreteCollisionDetectorInterface.Result {

                    val reportedNormalOnWorld = Vec3()
                    var reportedDistance = 0f
                    var foundResult = false

                    override fun setShapeIdentifiersA(partId0: Int, index0: Int) = Unit
                    override fun setShapeIdentifiersB(partId1: Int, index1: Int) = Unit
                    override fun addContactPoint(normalOnBInWorld: Vec3, pointInWorld: Vec3, depth: Float) {
                        reportedDistance = depth
                        reportedNormalOnWorld put normalOnBInWorld

                        val adjustedPointB = pointInWorld - normalOnBInWorld * marginOnB
                        reportedDistance = depth + (marginOnA + marginOnB)
                        if (reportedDistance < 0f) foundResult = true
                        originalResult.addContactPoint(normalOnBInWorld, adjustedPointB, reportedDistance)
                    }
                }

                val dummy = DummyResult()

                // BoxShape is an exception: its vertices are created WITH margin so don't subtract it

                val min0Margin = if (min0.shapeType == Bnt.BOX_SHAPE_PROXYTYPE) 0f else min0.margin
                val min1Margin = if (min1.shapeType == Bnt.BOX_SHAPE_PROXYTYPE) 0f else min1.margin

                val withoutMargin = WithoutMarginResult(resultOut, min0Margin, min1Margin)

                val polyhedronA = min0 as PolyhedralConvexShape
                val polyhedronB = min1 as PolyhedralConvexShape
                if (polyhedronA.polyhedron != null && polyhedronB.polyhedron != null) {

                    val threshold = manifold!!.contactBreakingThreshold

                    var minDist = -1e30f
                    val sepNormalWorldSpace = Vec3()
                    var foundSepAxis = true

                    if (dispatchInfo.enableSatConvex) {
                        foundSepAxis = PolyhedralContactClipping.findSeparatingAxis(polyhedronA.polyhedron!!, polyhedronB.polyhedron!!,
                                body0.worldTransform, body1.worldTransform, sepNormalWorldSpace, resultOut)
                    } else {
                        if (ZERO_MARGIN) {
                            gjkPairDetector.ignoreMargin = true
                            gjkPairDetector.getClosestPoints(input, resultOut, dispatchInfo.debugDraw)
                        } else {
                            gjkPairDetector.getClosestPoints(input, withoutMargin, dispatchInfo.debugDraw)
                            //gjkPairDetector.getClosestPoints(input,dummy,dispatchInfo.m_debugDraw);
                        }
                        //btScalar l2 = gjkPairDetector.getCachedSeparatingAxis().length2();
                        //if (l2>SIMD_EPSILON)
                        run {
                            sepNormalWorldSpace put withoutMargin.reportedNormalOnWorld //gjkPairDetector.getCachedSeparatingAxis()*(1.f/l2);
                            //minDist = -1e30f;//gjkPairDetector.getCachedSeparatingDistance();
                            minDist = withoutMargin.reportedDistance//gjkPairDetector.getCachedSeparatingDistance()+min0->getMargin()+min1->getMargin();
                            if (ZERO_MARGIN) foundSepAxis = true//gjkPairDetector.getCachedSeparatingDistance()<0.f;
                            else foundSepAxis = withoutMargin.foundResult && minDist < 0//-(min0->getMargin()+min1->getMargin());
                        }
                    }
                    if (foundSepAxis) {
//				printf("sepNormalWorldSpace=%f,%f,%f\n",sepNormalWorldSpace.getX(),sepNormalWorldSpace.getY(),sepNormalWorldSpace.getZ());
                        worldVertsB1.clear()
                        PolyhedralContactClipping.clipHullAgainstHull(sepNormalWorldSpace, polyhedronA.polyhedron!!,
                                polyhedronB.polyhedron!!, body0.worldTransform, body1.worldTransform,
                                minDist - threshold, threshold, worldVertsB1, worldVertsB2, resultOut)
                    }
                    if (ownManifold) resultOut.refreshContactPoints()
                    return
                } else {
                    //we can also deal with convex versus triangle (without connectivity data)
                    if (polyhedronA.polyhedron != null && polyhedronB.shapeType == Bnt.TRIANGLE_SHAPE_PROXYTYPE) {

                        val tri = polyhedronB as TriangleShape
                        val vertices = arrayListOf(body1.worldTransform * tri.vertices[0],
                                body1.worldTransform * tri.vertices[1], body1.worldTransform * tri.vertices[2])

                        //tri->initializePolyhedralFeatures();

                        val threshold = manifold!!.contactBreakingThreshold

                        val sepNormalWorldSpace = Vec3()
                        var minDist = -1e30f
                        val maxDist = threshold

                        var foundSepAxis = false
                        if (false) {
                            polyhedronB.initializePolyhedralFeatures()
                            foundSepAxis = PolyhedralContactClipping.findSeparatingAxis(polyhedronA.polyhedron!!, polyhedronB.polyhedron!!,
                                    body0.worldTransform, body1.worldTransform, sepNormalWorldSpace, resultOut)
                            //	 printf("sepNormalWorldSpace=%f,%f,%f\n",sepNormalWorldSpace.getX(),sepNormalWorldSpace.getY(),sepNormalWorldSpace.getZ());
                        } else {
                            if (ZERO_MARGIN) {
                                gjkPairDetector.ignoreMargin = true
                                gjkPairDetector.getClosestPoints(input, resultOut, dispatchInfo.debugDraw)
                            } else
                                gjkPairDetector.getClosestPoints(input, dummy, dispatchInfo.debugDraw)

                            val l2 = gjkPairDetector.cachedSeparatingAxis.length2()
                            if (l2 > Float.EPSILON) {
                                sepNormalWorldSpace put gjkPairDetector.cachedSeparatingAxis * (1f / l2)
                                //minDist = gjkPairDetector.getCachedSeparatingDistance();
                                //maxDist = threshold;
                                minDist = gjkPairDetector.cachedSeparatingDistance - min0.margin - min1.margin
                                foundSepAxis = true
                            }
                        }
                        if (foundSepAxis) {
                            worldVertsB2.clear()
                            PolyhedralContactClipping.clipFaceAgainstHull(sepNormalWorldSpace, polyhedronA.polyhedron!!,
                                    body0.worldTransform, vertices, worldVertsB2, minDist - threshold, maxDist, resultOut)
                        }
                        if (ownManifold) resultOut.refreshContactPoints()
                        return
                    }
                }
            }
            gjkPairDetector.getClosestPoints(input, resultOut, dispatchInfo.debugDraw)

            //now perform 'm_numPerturbationIterations' collision queries with the perturbated collision objects

            //perform perturbation when more then 'm_minimumPointsPerturbationThreshold' points
            if (numPerturbationIterations != 0 && resultOut.manifold!!.numContacts < minimumPointsPerturbationThreshold) {

                val v0 = Vec3()
                val v1 = Vec3()
                val l2 = gjkPairDetector.cachedSeparatingAxis.length2()

                if (l2 > Float.EPSILON) {
                    val sepNormalWorldSpace = gjkPairDetector.cachedSeparatingAxis * (1f / l2)

                    planeSpace1(sepNormalWorldSpace, v0, v1)

                    val perturbeA: Boolean
                    val angleLimit = 0.125f * PI
                    var perturbeAngle: Float
                    val radiusA = min0.angularMotionDisc
                    val radiusB = min1.angularMotionDisc
                    if (radiusA < radiusB) {
                        perturbeAngle = gContactBreakingThreshold / radiusA
                        perturbeA = true
                    } else {
                        perturbeAngle = gContactBreakingThreshold / radiusB
                        perturbeA = false
                    }
                    if (perturbeAngle > angleLimit) perturbeAngle = angleLimit

                    val unPerturbedTransform = if (perturbeA) input.transformA else input.transformB

                    for (i in 0 until numPerturbationIterations) {
                        if (v0.length2() > Float.EPSILON) {
                            val perturbeRot = Quat(v0, perturbeAngle)
                            val iterationAngle = i * (PI2 / numPerturbationIterations)
                            val rotq = Quat(sepNormalWorldSpace, iterationAngle)

                            if (perturbeA) {
                                input.transformA.basis = Mat3(rotq.inverse() * perturbeRot * rotq) * body0.worldTransform.basis
                                input.transformB put body1.worldTransform
                                if (DEBUG_CONTACTS) TODO()
//                                    dispatchInfo.debugDraw.drawTransform(input.transformA, 10.0)
                            } else {
                                input.transformA put body0.worldTransform
                                input.transformB.basis = Mat3(rotq.inverse() * perturbeRot * rotq) * body1.worldTransform.basis
                                if (DEBUG_CONTACTS) TODO()
//                                        dispatchInfo.m_debugDraw->drawTransform(input.m_transformB, 10.0)
                            }
                            val perturbedResultOut = PerturbedContactResult(resultOut, input.transformA, input.transformB,
                                    unPerturbedTransform, perturbeA, dispatchInfo.debugDraw!!)
                            gjkPairDetector.getClosestPoints(input, perturbedResultOut, dispatchInfo.debugDraw!!)
                        }
                    }
                }
            }
            if (USE_SEPDISTANCE_UTIL2 && dispatchInfo.useConvexConservativeDistanceUtil && sepDist > Float.EPSILON)
                sepDistance!!.initSeparatingDistance(gjkPairDetector.cachedSeparatingAxis, sepDist, body0.worldTransform, body1.worldTransform)
        }
        if (ownManifold) resultOut.refreshContactPoints()
    }


    override fun calculateTimeOfImpact(body0: CollisionObject, body1: CollisionObject, dispatchInfo: DispatcherInfo, resultOut: ManifoldResult): Float {
        // Rather then checking ALL pairs, only calculate TOI when motion exceeds threshold

        // Linear motion for one of objects needs to exceed m_ccdSquareMotionThreshold
        // col0->m_worldTransform,
        var resultFraction = 1f

        val squareMot0 = (body0.getInterpolationWorldTransform().origin - body0.getWorldTransform().origin).length2()
        val squareMot1 = (body1.getInterpolationWorldTransform().origin - body1.getWorldTransform().origin).length2()

        if (squareMot0 < body0.ccdSquareMotionThreshold && squareMot1 < body1.ccdSquareMotionThreshold)
            return resultFraction

        if (disableCcd) return 1f

        /*  An adhoc way of testing the Continuous Collision Detection algorithms
            One object is approximated as a sphere, to simplify things
            Starting in penetration should report no time of impact
            For proper CCD, better accuracy and handling of 'allowed' penetration should be added
            also the mainloop of the physics should have a kind of toi queue (something like Brian Mirtich's application
            of Timewarp for Rigidbodies) */

        /// Convex0 against sphere for Convex1
        run {
            val convex0 = body0.collisionShape as ConvexShape

            val sphere1 = SphereShape(body1.ccdSweptSphereRadius) //todo: allow non-zero sphere sizes, for better approximation
            val result = ConvexCast.CastResult()
            val voronoiSimplex = VoronoiSimplexSolver()
            //SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
            ///Simplification, one object is simplified as a sphere
            val ccd1 = GjkConvexCast(convex0, sphere1, voronoiSimplex)
            //ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
            if (ccd1.calcTimeOfImpact(body0.getWorldTransform(), body0.getInterpolationWorldTransform(),
                            body1.getWorldTransform(), body1.getInterpolationWorldTransform(), result)) {

                //store result.m_fraction in both bodies

                if (body0.hitFraction > result.fraction) body0.hitFraction = result.fraction
                if (body1.hitFraction > result.fraction) body1.hitFraction = result.fraction

                if (resultFraction > result.fraction) resultFraction = result.fraction
            }
        }
        // Sphere (for convex0) against Convex1
        run {
            val convex1 = body1.collisionShape as ConvexShape

            val sphere0 = SphereShape(body0.ccdSweptSphereRadius) //todo: allow non-zero sphere sizes, for better approximation
            val result = ConvexCast.CastResult()
            val voronoiSimplex = VoronoiSimplexSolver()
            //SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
            ///Simplification, one object is simplified as a sphere
            val ccd1 = GjkConvexCast(sphere0, convex1, voronoiSimplex)
            //ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
            if (ccd1.calcTimeOfImpact(body0.getWorldTransform(), body0.getInterpolationWorldTransform(),
                            body1.getWorldTransform(), body1.getInterpolationWorldTransform(), result)) {

                //store result.m_fraction in both bodies

                if (body0.hitFraction > result.fraction) body0.hitFraction = result.fraction
                if (body0.hitFraction > result.fraction) body1.hitFraction = result.fraction

                if (resultFraction > result.fraction) resultFraction = result.fraction
            }
        }
        return resultFraction
    }

    override fun getAllContactManifolds(manifoldArray: ArrayList<PersistentManifold>) {
        // should we use m_ownManifold to avoid adding duplicates?
        manifold?.let { if (ownManifold) manifoldArray.add(it) }
    }

    class CreateFunc(val solver: ConvexPenetrationDepthSolver) : CollisionAlgorithmCreateFunc() {

        var numPerturbationIterations = 0
        var minimumPointsPerturbationThreshold = 3

        override fun createCollisionAlgorithm(info: CollisionAlgorithmConstructionInfo, body0Wrap: CollisionObjectWrapper,
                                              body1Wrap: CollisionObjectWrapper) =
//          void* mem = info.dispatcher!!.allocateCollisionAlgorithm(sizeof(btConvexConvexAlgorithm))
                ConvexConvexAlgorithm(info.manifold, info, body0Wrap, body1Wrap, solver, numPerturbationIterations, minimumPointsPerturbationThreshold)
    }

    class PerturbedContactResult(val originalManifoldResult: ManifoldResult, val transformA: Transform, val transformB: Transform,
                                 val unPerturbedTransform: Transform, val perturbA: Boolean, val debugDrawer: DebugDraw) :
            ManifoldResult() {

        override fun addContactPoint(normalOnBInWorld: Vec3, pointInWorld: Vec3, depth: Float) {

            val endPt: Vec3
            val startPt: Vec3
            val newDepth: Float
            val newNormal = Vec3()
            if (perturbA) {
                val endPtOrg = pointInWorld + normalOnBInWorld * depth
                endPt = (unPerturbedTransform * transformA.inverse())(endPtOrg)
                newDepth = (endPt - pointInWorld).dot(normalOnBInWorld)
                startPt = endPt - normalOnBInWorld * newDepth
            } else {
                endPt = pointInWorld + normalOnBInWorld * depth
                startPt = (unPerturbedTransform * transformB.inverse())(pointInWorld)
                newDepth = (endPt - startPt).dot(normalOnBInWorld)
            }

            if (DEBUG_CONTACTS) {
                TODO()
//                m_debugDrawer ->
//                drawLine(startPt, endPt, btVector3(1, 0, 0))
//                m_debugDrawer->drawSphere(startPt, 0.05, btVector3(0, 1, 0))
//                m_debugDrawer->drawSphere(endPt, 0.05, btVector3(0, 0, 1))
            }
            originalManifoldResult.addContactPoint(normalOnBInWorld, startPt, newDepth)
        }
    }

    companion object {

        fun capsuleCapsuleDistance(normalOnB: Vec3, pointOnB: Vec3, capsuleLengthA: Float, capsuleRadiusA: Float,
                                   capsuleLengthB: Float, capsuleRadiusB: Float, capsuleAxisA: Int, capsuleAxisB: Int,
                                   transformA: Transform, transformB: Transform, distanceThreshold: Float): Float {
            val directionA = transformA.basis.getColumn(capsuleAxisA)
            val translationA = transformA.origin
            val directionB = transformB.basis.getColumn(capsuleAxisB)
            val translationB = transformB.origin

            // translation between centers

            val translation = translationB - translationA

            // compute the closest points of the capsule line segments

            val ptsVector = Vec3() // the vector between the closest points

            val offsetA = Vec3() // offsets from segment centers to their closest points
            val offsetB = Vec3()
            // parameters on line segment
            val (tA, tB) = segmentsClosestPoints(ptsVector, offsetA, offsetB, translation, directionA, capsuleLengthA, directionB, capsuleLengthB)

            val distance = ptsVector.length() - capsuleRadiusA - capsuleRadiusB

            if (distance > distanceThreshold) return distance

            val lenSqr = ptsVector.length2()
            //degenerate case where 2 capsules are likely at the same location: take a vector tangential to 'directionA'
            if (lenSqr <= Float.EPSILON * Float.EPSILON)
                planeSpace1(directionA, normalOnB, Vec3())
            else // compute the contact normal
                normalOnB put ptsVector * -recipSqrt(lenSqr)
            pointOnB put transformB.origin + offsetB + normalOnB * capsuleRadiusB

            return distance
        }

        /** JVM specific,
         *  @return tA tB */
        fun segmentsClosestPoints(ptsVector: Vec3, offsetA: Vec3, offsetB: Vec3, translation: Vec3, dirA: Vec3, hlenA: Float,
                                  dirB: Vec3, hlenB: Float): FloatArray {
            // compute the parameters of the closest points on each line segment
            val dirA_dot_dirB = dirA dot dirB
            val dirA_dot_trans = dirA dot translation
            val dirB_dot_trans = dirB dot translation

            val denom = 1f - dirA_dot_dirB * dirA_dot_dirB

            var tA = when (denom) {
                0f -> 0f
                else -> clamped((dirA_dot_trans - dirB_dot_trans * dirA_dot_dirB), -hlenA, hlenA)
            }

            val tB = clamped(tA * dirA_dot_dirB - dirB_dot_trans, -hlenB, hlenB)
            tA = clamped(tB * dirA_dot_dirB + dirA_dot_trans, -hlenA, hlenA)

            // compute the closest points relative to segment centers.

            val offsetA = dirA * tA
            val offsetB = dirB * tB

            ptsVector put translation - offsetA + offsetB // TODO check

            return floatArrayOf(tA, tB)
        }
    }
}