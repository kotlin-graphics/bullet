/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011 Advanced Micro Devices, Inc.  http://bulletphysics.org

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

import bullet.EPSILON
import bullet.ONLY_REPORT_DEEPEST_POINT
import bullet.TEST_INTERNAL_OBJECTS
import bullet.collision.collisionDispatch.ConvexConvexAlgorithm
import bullet.collision.collisionShapes.ConvexPolyhedron
import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import bullet.linearMath.min
import bullet.resize
import kotlin.math.sqrt
import kotlin.reflect.KMutableProperty0

var gActualSATPairTests = 0
var gExpectedNbTests = 0
var gActualNbTests = 0
var gUseInternalObject = true

// Clips a face to the back of a plane
object PolyhedralContactClipping {

    fun clipHullAgainstHull(separatingNormal1: Vec3, hullA: ConvexPolyhedron, hullB: ConvexPolyhedron, transA: Transform,
                            transB: Transform, minDist: Float, maxDist: Float, worldVertsB1: ArrayList<Vec3>,
                            worldVertsB2: ArrayList<Vec3>, resultOut: DiscreteCollisionDetectorInterface.Result) {

        val separatingNormal = separatingNormal1.normalized()

        var closestFaceB = -1
        var dmax = -Float.MAX_VALUE

        hullB.faces.forEachIndexed { i, face ->
            val normal = Vec3(face.plane)
            val worldNormal = transB.basis * normal
            val d = worldNormal dot separatingNormal
            if (d > dmax) {
                dmax = d
                closestFaceB = i
            }
        }

        worldVertsB1.clear()

        val polyB = hullB.faces[closestFaceB]
        val numVertices = polyB.indices.size
        repeat(numVertices) {
            val b = hullB.vertices[polyB.indices[it]]
            worldVertsB1.add(transB * b)
        }

        if (closestFaceB >= 0)
            clipFaceAgainstHull(separatingNormal, hullA, transA, worldVertsB1, worldVertsB2, minDist, maxDist, resultOut)
    }

    fun clipFaceAgainstHull(separatingNormal: Vec3, hullA: ConvexPolyhedron, transA: Transform, worldVertsB1: ArrayList<Vec3>,
                            worldVertsB2: ArrayList<Vec3>, minDist: Float, maxDist: Float,
                            resultOut: DiscreteCollisionDetectorInterface.Result) {
        worldVertsB2.clear()
        val vtxIn = worldVertsB1
        val vtxOut = worldVertsB2
        vtxOut.resize(vtxIn.size)

        var closestFaceA = -1

        var dmin = Float.MAX_VALUE
        hullA.faces.forEachIndexed { i, face ->
            val normal = Vec3(face.plane)
            val faceANormalWS = transA.basis * normal
            val d = faceANormalWS dot separatingNormal
            if (d < dmin) {
                dmin = d
                closestFaceA = i
            }
        }

        if (closestFaceA < 0) return

        val polyA = hullA.faces[closestFaceA]

        // clip polygon to back of planes of all faces of hull A that are adjacent to witness face
        val numVerticesA = polyA.indices.size
        repeat(numVerticesA) {
            val a = hullA.vertices[polyA.indices[it]]
            val b = hullA.vertices[polyA.indices[(it + 1) % numVerticesA]]
            val edge0 = a - b
            val worldEdge0 = transA.basis * edge0
            val worldPlaneAnormal1 = transA.basis * Vec3(polyA.plane)

            val planeNormalWS1 = -worldEdge0 cross worldPlaneAnormal1 //.cross(worldEdge0);
            val worldA1 = transA * a
            val planeEqWS1 = -worldA1 dot planeNormalWS1

//int otherFace=0;
//            #ifdef BLA1  TODO?
//                int otherFace = polyA . m_connectedFaces [e0]
//            btVector3 localPlaneNormal (hullA.m_faces[otherFace].m_plane[0], hullA.m_faces[otherFace].m_plane[1], hullA.m_faces[otherFace].m_plane[2])
//            btScalar localPlaneEq = hullA . m_faces [otherFace].m_plane[3]
//
//            btVector3 planeNormalWS = transA . getBasis () * localPlaneNormal
//            btScalar planeEqWS = localPlaneEq -planeNormalWS.dot(transA.getOrigin())
//            #else
            val planeNormalWS = planeNormalWS1
            val planeEqWS = planeEqWS1
//            #endif
            //clip face
            clipFace(vtxIn, vtxOut, planeNormalWS, planeEqWS)
            vtxIn[0] = vtxOut[0]
            vtxOut.clear()
        }

        val point = Vec3()

        // only keep points that are behind the witness face
        val localPlaneNormal = Vec3(polyA.plane)
        val localPlaneEq = polyA.plane[3]
        val planeNormalWS = transA.basis * localPlaneNormal
        val planeEqWS = localPlaneEq - (planeNormalWS dot transA.origin)
        vtxIn.forEachIndexed { i, vtx ->
            var depth = (planeNormalWS dot vtx) + planeEqWS
            if (depth <= minDist) {
//				printf("clamped: depth=%f to minDist=%f\n",depth,minDist);
                depth = minDist
            }
            if (depth <= maxDist) {
                val point = vtxIn[i]
//                if (ONLY_REPORT_DEEPEST_POINT) TODO no def of curMaxDist
//                    curMaxDist = depth
//                else if (false && depth < -3) {
//                        printf("error in btPolyhedralContactClipping depth = %f\n", depth)
//                        printf("likely wrong separatingNormal passed in\n")
//                    }
                resultOut.addContactPoint(separatingNormal, point, depth)
            }
        }
//        if(ONLY_REPORT_DEEPEST_POINT)
//                if (curMaxDist < maxDist) {
//                    resultOut.addContactPoint(separatingNormal, point, curMaxDist)
//                }
    }


    private var d = 0f

    fun findSeparatingAxis(hullA: ConvexPolyhedron, hullB: ConvexPolyhedron, transA: Transform, transB: Transform,
                           sep: Vec3, resultOut: DiscreteCollisionDetectorInterface.Result): Boolean {
        gActualSATPairTests++

        val c0 = transA * hullA.localCenter
        val c1 = transB * hullB.localCenter
        val deltaC2 = c0 - c1

        var dmin = Float.MAX_VALUE
        var curPlaneTests = 0

        val numFacesA = hullA.faces.size
        // Test normals from hullA
        for (i in 0 until numFacesA) {
            val normal = Vec3(hullA.faces[i].plane)
            val faceANormalWS = transA.basis * normal
            if (deltaC2 dot faceANormalWS < 0) faceANormalWS *= -1f
            curPlaneTests++
            if (TEST_INTERNAL_OBJECTS) {
                gExpectedNbTests++
                if (gUseInternalObject && !testInternalObjects(transA, transB, deltaC2, faceANormalWS, hullA, hullB, dmin))
                    continue
                gActualNbTests++
            }

            val wA = Vec3()
            val wB = Vec3()
            if (!testSepAxis(hullA, hullB, transA, transB, faceANormalWS, ::d, wA, wB)) return false

            if (d < dmin) {
                dmin = d
                sep put faceANormalWS
            }
        }

        val numFacesB = hullB.faces.size
        // Test normals from hullB
        for (i in 0 until numFacesB) {
            val normal = Vec3(hullB.faces[i].plane)
            val worldNormal = transB.basis * normal
            if (deltaC2 dot worldNormal < 0) worldNormal *= -1f
            curPlaneTests++
            if (TEST_INTERNAL_OBJECTS) {
                gExpectedNbTests++
                if (gUseInternalObject && !testInternalObjects(transA, transB, deltaC2, worldNormal, hullA, hullB, dmin))
                    continue
                gActualNbTests++
            }

            val wA = Vec3()
            val wB = Vec3()
            if (!testSepAxis(hullA, hullB, transA, transB, worldNormal, ::d, wA, wB)) return false

            if (d < dmin) {
                dmin = d
                sep put worldNormal
            }
        }

        val edgeAstart = Vec3()
        val edgeAend = Vec3()
        val edgeBstart = Vec3()
        val edgeBend = Vec3()
        var edgeA = -1
        var edgeB = -1
        val worldEdgeA = Vec3()
        val worldEdgeB = Vec3()
        val witnessPointA = Vec3()
        val witnessPointB = Vec3()

        var curEdgeEdge = 0
        // Test edges
        for (e0 in 0 until hullA.uniqueEdges.size) {
            val edge0 = hullA.uniqueEdges[e0]
            val worldEdge0 = transA.basis * edge0
            for (e1 in 0 until hullB.uniqueEdges.size) {
                val edge1 = hullB.uniqueEdges[e1]
                val worldEdge1 = transB.basis * edge1

                val cross = worldEdge0 cross worldEdge1
                curEdgeEdge++
                if (!cross.isAlmostZero) {
                    cross.normalize()
                    if (deltaC2.dot(cross) < 0) cross *= -1f

                    if (TEST_INTERNAL_OBJECTS) {
                        gExpectedNbTests++
                        if (gUseInternalObject && !testInternalObjects(transA, transB, deltaC2, cross, hullA, hullB, dmin))
                            continue
                        gActualNbTests++
                    }

                    val wA = Vec3()
                    val wB = Vec3()
                    if (!testSepAxis(hullA, hullB, transA, transB, cross, ::d, wA, wB)) return false

                    if (d < dmin) {
                        dmin = d
                        sep put cross
                        edgeA = e0
                        edgeB = e1
                        worldEdgeA put worldEdge0
                        worldEdgeB put worldEdge1
                        witnessPointA put wA
                        witnessPointB put wB
                    }
                }
            }

        }

        if (edgeA >= 0 && edgeB >= 0) {
//		printf("edge-edge\n");
            //add an edge-edge contact
            val ptsVector = Vec3()
            val offsetA = Vec3()
            val offsetB = Vec3()

            val translation = witnessPointB - witnessPointA

            val dirA = worldEdgeA
            val dirB = worldEdgeB

            val hlenB = 1e30f
            val hlenA = 1e30f

            val (tA, tB) = ConvexConvexAlgorithm.segmentsClosestPoints(ptsVector, offsetA, offsetB, translation, dirA, hlenA, dirB, hlenB)

            val nlSqrt = ptsVector.length2()
            if (nlSqrt > Float.EPSILON) {
                val nl = sqrt(nlSqrt)
                ptsVector *= 1f / nl
                if (ptsVector dot deltaC2 < 0f) ptsVector *= -1f
                val ptOnB = witnessPointB + offsetB
                val distance = nl
                resultOut.addContactPoint(ptsVector, ptOnB, -distance)
            }
        }
        if (deltaC2 dot sep < 0f) sep put -sep
        return true
    }

    fun testInternalObjects(trans0: Transform, trans1: Transform, deltaC: Vec3, axis: Vec3, convex0: ConvexPolyhedron,
                            convex1: ConvexPolyhedron, dmin: Float): Boolean {

        val dp = deltaC dot axis

        val localAxis0 = axis inverseTransformPoint3x3 trans0
        val localAxis1 = axis inverseTransformPoint3x3 trans1

        val p0 = convex0.extents boxSupport localAxis0
        val p1 = convex1.extents boxSupport localAxis1

        val radius0 = p0.x * localAxis0.x + p0.y * localAxis0.y + p0.z * localAxis0.z
        val radius1 = p1.x * localAxis1.x + p1.y * localAxis1.y + p1.z * localAxis1.z

        val minRadius = if (radius0 > convex0.radius) radius0 else convex0.radius
        val maxRadius = if (radius1 > convex1.radius) radius1 else convex1.radius

        val minMaxRadius = maxRadius + minRadius
        val d0 = minMaxRadius + dp
        val d1 = minMaxRadius - dp

        val depth = d0 min d1
        return depth <= dmin
    }

    /** JVM specif, out in return */
    infix fun Vec3.inverseTransformPoint3x3(tr: Transform): Vec3 {
        val r = tr.basis
        return Vec3(
                r[0, 0] * x + r[1, 0] * y + r[2, 0] * z,
                r[0, 1] * x + r[1, 1] * y + r[2, 1] * z,
                r[0, 2] * x + r[1, 2] * y + r[3, 2] * z)
    }

    /** JVM specif, p in return */
    infix fun Vec3.boxSupport(sv: Vec3): Vec3 {
        // This version is ~11.000 cycles (4%) faster overall in one of the tests.
//	IR(p[0]) = IR(extents[0])|(IR(sv[0])&SIGN_BITMASK);
//	IR(p[1]) = IR(extents[1])|(IR(sv[1])&SIGN_BITMASK);
//	IR(p[2]) = IR(extents[2])|(IR(sv[2])&SIGN_BITMASK);
        return Vec3(
                if (sv.x < 0f) -x else x,
                if (sv.y < 0f) -y else y,
                if (sv.z < 0f) -z else z)
    }

    fun testSepAxis(hullA: ConvexPolyhedron, hullB: ConvexPolyhedron, transA: Transform, transB: Transform, sepAxis: Vec3,
                    depth: KMutableProperty0<Float>, witnessPointA: Vec3, witnessPointB: Vec3): Boolean {
        val min = 0
        val max = 1
        val mm0 = FloatArray(2)
        val mm1 = FloatArray(2)
        val witnesPtMinA = Vec3()
        val witnesPtMaxA = Vec3()
        val witnesPtMinB = Vec3()
        val witnesPtMaxB = Vec3()

        hullA.project(transA, sepAxis, mm0, witnesPtMinA, witnesPtMaxA)
        hullB.project(transB, sepAxis, mm1, witnesPtMinB, witnesPtMaxB)

        return if (mm0[max] < mm1[min] || mm1[max] < mm0[min]) false
        else {
            val d0 = mm0[max] - mm1[min]
            assert(d0 >= 0f)
            val d1 = mm1[max] - mm0[min]
            assert(d1 >= 0f)
            if (d0 < d1) {
                depth.set(d0)
                witnessPointA put witnesPtMaxA
                witnessPointB put witnesPtMinB
            } else {
                depth.set(d1)
                witnessPointA put witnesPtMinA
                witnessPointB put witnesPtMaxB
            }
            true
        }
    }

    /** Clips a face to the back of a plane, the clipFace method is used internally */
    fun clipFace(vtxIn: ArrayList<Vec3>, vtxOut: ArrayList<Vec3>, planeNormalWS: Vec3, planeEqWS: Float) {

        val numVerts = vtxIn.size
        if (numVerts < 2) return

        var firstVertex = vtxIn.last()
        var endVertex = vtxIn[0]

        var ds = (planeNormalWS dot firstVertex) + planeEqWS

        repeat(numVerts) {
            endVertex = vtxIn[it]

            val de = (planeNormalWS dot endVertex) + planeEqWS

            if (ds < 0)
                if (de < 0) // Start < 0, end < 0, so output endVertex
                    vtxOut.add(endVertex)
                else  // Start < 0, end >= 0, so output intersection
                    vtxOut.add(firstVertex.lerp(endVertex, ds * 1f / (ds - de)))
            else if (de < 0) { // Start >= 0, end < 0 so output intersection and end
                vtxOut.add(firstVertex.lerp(endVertex, ds * 1f / (ds - de)))
                vtxOut.add(endVertex)
            }
            firstVertex = endVertex
            ds = de
        }
    }
}
