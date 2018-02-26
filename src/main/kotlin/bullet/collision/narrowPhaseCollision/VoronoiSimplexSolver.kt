package bullet.collision.narrowPhaseCollision

import bullet.linearMath.LARGE_FLOAT
import bullet.linearMath.Vec3
import bullet.linearMath.times


val VERTA = 0
val VERTB = 1
val VERTC = 2
val VERTD = 3

val CATCH_DEGENERATE_TETRAHEDRON = true

val VORONOI_SIMPLEX_MAX_VERTS = 5
///disable next define, or use defaultCollisionConfiguration->getSimplexSolver()->setEqualVertexThreshold(0.f) to disable/configure
val USE_EQUAL_VERTEX_THRESHOLD = true
val VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD = 0.0001f

class UsageBitfield {

    var usedVertexA = false
    var usedVertexB = false
    var usedVertexC = false
    var usedVertexD = false
    var unused1 = false
    var unused2 = false
    var unused3 = false
    var unused4 = false

    fun reset() {
        usedVertexA = false
        usedVertexB = false
        usedVertexC = false
        usedVertexD = false
    }
}

class SubSimplexClosestResult {

    val closestPointOnSimplex = Vec3()

    /** MASK for usedVertices
     *  stores the simplex vertex-usage, using the MASK, if usedVertices & MASK then the related vertex is used */
    val usedVertices = UsageBitfield()
    val barycentricCoords = FloatArray(4)
    var degenerate = false

    fun reset() {
        degenerate = false
        setBarycentricCoordinates()
        usedVertices.reset()
    }

    val isValid get() = barycentricCoords[0] >= 0f && barycentricCoords[1] >= 0f && barycentricCoords[2] >= 0f && barycentricCoords[3] >= 0f

    fun setBarycentricCoordinates(a: Float = 0f, b: Float = 0f, c: Float = 0f, d: Float = 0f) {
        barycentricCoords[0] = a
        barycentricCoords[1] = b
        barycentricCoords[2] = c
        barycentricCoords[3] = d
    }
}

/** VoronoiSimplexSolver is an implementation of the closest point distance algorithm from a 1-4 points simplex
 *  to the origin.
 *  Can be used with GJK, as an alternative to Johnson distance algorithm.  */
class VoronoiSimplexSolver {

    var numVertices = 0

    val simplexVectorW = Array(VORONOI_SIMPLEX_MAX_VERTS, { Vec3() })
    val simplexPointsP = Array(VORONOI_SIMPLEX_MAX_VERTS, { Vec3() })
    val simplexPointsQ = Array(VORONOI_SIMPLEX_MAX_VERTS, { Vec3() })

    var cachedP1 = Vec3()
    var cachedP2 = Vec3()
    var cachedV = Vec3()
    var lastW = Vec3()

    var equalVertexThreshold = VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD
    var cachedValidClosest = false

    val cachedBC = SubSimplexClosestResult()

    var needsUpdate = false


    fun removeVertex(index: Int) {
        assert(numVertices > 0)
        numVertices--
        simplexVectorW[index] put simplexVectorW[numVertices]
        simplexPointsP[index] put simplexPointsP[numVertices]
        simplexPointsQ[index] put simplexPointsQ[numVertices]
    }

    fun reduceVertices(usedVerts: UsageBitfield) {
        if (numVertices >= 4 && !usedVerts.usedVertexD) removeVertex(3)
        if (numVertices >= 3 && !usedVerts.usedVertexC) removeVertex(2)
        if (numVertices >= 2 && !usedVerts.usedVertexB) removeVertex(1)
        if (numVertices >= 1 && !usedVerts.usedVertexA) removeVertex(0)
    }

    fun updateClosestVectorAndPoints(): Boolean {

        if (needsUpdate) {

            cachedBC.reset()

            needsUpdate = false

            when (numVertices) {
                0 -> cachedValidClosest = false
                1 -> {
                    cachedP1 put simplexPointsP[0]
                    cachedP2 put simplexPointsQ[0]
                    //== m_simplexVectorW[0]
                    cachedV = cachedP1 - cachedP2
                    cachedBC.reset()
                    cachedBC.setBarycentricCoordinates(1f, 0f, 0f, 0f)
                    cachedValidClosest = cachedBC.isValid
                }
                2 -> {
                    //closest point origin from line segment
                    val from = simplexVectorW[0]
                    val to = simplexVectorW[1]

                    val p = Vec3()
                    val diff = p - from
                    val v = to - from
                    var t = v dot diff

                    if (t > 0) {
                        val dotVV = v dot v
                        if (t < dotVV) {
                            t /= dotVV
                            diff -= t * v
                            cachedBC.usedVertices.usedVertexA = true
                            cachedBC.usedVertices.usedVertexB = true
                        } else {
                            t = 1f
                            diff -= v
                            //reduce to 1 point
                            cachedBC.usedVertices.usedVertexB = true
                        }
                    } else {
                        t = 0f
                        //reduce to 1 point
                        cachedBC.usedVertices.usedVertexA = true
                    }
                    cachedBC.setBarycentricCoordinates(1 - t, t)
                    val nearest = from + t * v

                    cachedP1 = simplexPointsP[0] + t * (simplexPointsP[1] - simplexPointsP[0])
                    cachedP2 = simplexPointsQ[0] + t * (simplexPointsQ[1] - simplexPointsQ[0])
                    cachedV = cachedP1 - cachedP2

                    reduceVertices(cachedBC.usedVertices)

                    cachedValidClosest = cachedBC.isValid
                }
                3 -> {
                    //closest point origin from triangle
                    val p = Vec3()

                    val a = simplexVectorW[0]
                    val b = simplexVectorW[1]
                    val c = simplexVectorW[2]

                    closestPtPointTriangle(p, a, b, c, cachedBC)
                    cachedP1 = simplexPointsP[0] * cachedBC.barycentricCoords[0] +
                            simplexPointsP[1] * cachedBC.barycentricCoords[1] +
                            simplexPointsP[2] * cachedBC.barycentricCoords[2]

                    cachedP2 = simplexPointsQ[0] * cachedBC.barycentricCoords[0] +
                            simplexPointsQ[1] * cachedBC.barycentricCoords[1] +
                            simplexPointsQ[2] * cachedBC.barycentricCoords[2]

                    cachedV = cachedP1 - cachedP2

                    reduceVertices(cachedBC.usedVertices)
                    cachedValidClosest = cachedBC.isValid
                }
                4 -> {
                    val p = Vec3()

                    val a = simplexVectorW[0]
                    val b = simplexVectorW[1]
                    val c = simplexVectorW[2]
                    val d = simplexVectorW[3]

                    if (closestPtPointTetrahedron(p, a, b, c, d, cachedBC)) {   // has separation

                        cachedP1 = simplexPointsP[0] * cachedBC.barycentricCoords[0] +
                                simplexPointsP[1] * cachedBC.barycentricCoords[1] +
                                simplexPointsP[2] * cachedBC.barycentricCoords[2] +
                                simplexPointsP[3] * cachedBC.barycentricCoords[3]

                        cachedP2 = simplexPointsQ[0] * cachedBC.barycentricCoords[0] +
                                simplexPointsQ[1] * cachedBC.barycentricCoords[1] +
                                simplexPointsQ[2] * cachedBC.barycentricCoords[2] +
                                simplexPointsQ[3] * cachedBC.barycentricCoords[3]

                        cachedV = cachedP1 - cachedP2
                        reduceVertices(cachedBC.usedVertices)
                    } else {
//					printf("sub distance got penetration\n");

                        if (cachedBC.degenerate)
                            cachedValidClosest = false
                        else {
                            cachedValidClosest = true
                            //degenerate case == false, penetration = true + zero
                            cachedV put 0f
                        }
                    }
                    cachedValidClosest = cachedBC.isValid

                    //closest point origin from tetrahedron
                }
                else -> cachedValidClosest = false
            }
        }
        return cachedValidClosest
    }


    fun closestPtPointTetrahedron(p: Vec3, a: Vec3, b: Vec3, c: Vec3, d: Vec3, finalResult: SubSimplexClosestResult): Boolean {

        val tempResult = SubSimplexClosestResult()

        // Start out assuming point inside all halfspaces, so closest to itself
        finalResult.closestPointOnSimplex put p
        finalResult.usedVertices.reset()
        finalResult.usedVertices.usedVertexA = true
        finalResult.usedVertices.usedVertexB = true
        finalResult.usedVertices.usedVertexC = true
        finalResult.usedVertices.usedVertexD = true

        val pointOutsideABC = pointOutsideOfPlane(p, a, b, c, d)
        val pointOutsideACD = pointOutsideOfPlane(p, a, c, d, b)
        val pointOutsideADB = pointOutsideOfPlane(p, a, d, b, c)
        val pointOutsideBDC = pointOutsideOfPlane(p, b, d, c, a)

        if (pointOutsideABC < 0 || pointOutsideACD < 0 || pointOutsideADB < 0 || pointOutsideBDC < 0) {
            finalResult.degenerate = true
            return false
        }

        if (pointOutsideABC == 0 && pointOutsideACD == 0 && pointOutsideADB == 0 && pointOutsideBDC == 0)
            return false

        var bestSqDist = Float.MAX_VALUE
        // If point outside face abc then compute closest point on abc
        if (pointOutsideABC != 0) {
            closestPtPointTriangle(p, a, b, c, tempResult)
            val q = tempResult.closestPointOnSimplex

            val sqDist = (q - p) dot (q - p)
            // Update best closest point if (squared) distance is less than current best
            if (sqDist < bestSqDist) {
                bestSqDist = sqDist
                finalResult.closestPointOnSimplex put q
                //convert result bitmask!
                finalResult.usedVertices.reset()
                finalResult.usedVertices.usedVertexA = tempResult.usedVertices.usedVertexA
                finalResult.usedVertices.usedVertexB = tempResult.usedVertices.usedVertexB
                finalResult.usedVertices.usedVertexC = tempResult.usedVertices.usedVertexC
                finalResult.setBarycentricCoordinates(
                        tempResult.barycentricCoords[VERTA],
                        tempResult.barycentricCoords[VERTB],
                        tempResult.barycentricCoords[VERTC],
                        0f)
            }
        }


        // Repeat test for face acd
        if (pointOutsideACD != 0) {
            closestPtPointTriangle(p, a, c, d, tempResult)
            val q = tempResult.closestPointOnSimplex
            //convert result bitmask!

            val sqDist = (q - p) dot (q - p)
            if (sqDist < bestSqDist) {
                bestSqDist = sqDist
                finalResult.closestPointOnSimplex put q
                finalResult.usedVertices.reset()
                finalResult.usedVertices.usedVertexA = tempResult.usedVertices.usedVertexA

                finalResult.usedVertices.usedVertexC = tempResult.usedVertices.usedVertexB
                finalResult.usedVertices.usedVertexD = tempResult.usedVertices.usedVertexC
                finalResult.setBarycentricCoordinates(
                        tempResult.barycentricCoords[VERTA],
                        0f,
                        tempResult.barycentricCoords[VERTB],
                        tempResult.barycentricCoords[VERTC])
            }
        }
        // Repeat test for face adb


        if (pointOutsideADB != 0) {
            closestPtPointTriangle(p, a, d, b, tempResult)
            val q = tempResult.closestPointOnSimplex
            //convert result bitmask!

            val sqDist = (q - p) dot (q - p)
            if (sqDist < bestSqDist) {
                bestSqDist = sqDist
                finalResult.closestPointOnSimplex put q
                finalResult.usedVertices.reset()
                finalResult.usedVertices.usedVertexA = tempResult.usedVertices.usedVertexA
                finalResult.usedVertices.usedVertexB = tempResult.usedVertices.usedVertexC

                finalResult.usedVertices.usedVertexD = tempResult.usedVertices.usedVertexB
                finalResult.setBarycentricCoordinates(
                        tempResult.barycentricCoords[VERTA],
                        tempResult.barycentricCoords[VERTC],
                        0f,
                        tempResult.barycentricCoords[VERTB])
            }
        }
        // Repeat test for face bdc


        if (pointOutsideBDC != 0) {
            closestPtPointTriangle(p, b, d, c, tempResult)
            val q = tempResult.closestPointOnSimplex
            //convert result bitmask!
            val sqDist = (q - p) dot (q - p)
            if (sqDist < bestSqDist) {
                bestSqDist = sqDist
                finalResult.closestPointOnSimplex put q
                finalResult.usedVertices.reset()
                //
                finalResult.usedVertices.usedVertexB = tempResult.usedVertices.usedVertexA
                finalResult.usedVertices.usedVertexC = tempResult.usedVertices.usedVertexC
                finalResult.usedVertices.usedVertexD = tempResult.usedVertices.usedVertexB

                finalResult.setBarycentricCoordinates(
                        0f,
                        tempResult.barycentricCoords[VERTA],
                        tempResult.barycentricCoords[VERTC],
                        tempResult.barycentricCoords[VERTB])
            }
        }
        //help! we ended up full !
        if (finalResult.usedVertices.usedVertexA && finalResult.usedVertices.usedVertexB &&
                finalResult.usedVertices.usedVertexC && finalResult.usedVertices.usedVertexD)
            return true

        return true
    }

    /** Test if point p and d lie on opposite sides of plane through abc    */
    fun pointOutsideOfPlane(p: Vec3, a: Vec3, b: Vec3, c: Vec3, d: Vec3): Int {

        val normal = (b - a) cross (c - a)

        val signp = (p - a) dot normal // [AP AB AC]
        val signd = (d - a) dot normal // [AD AB AC]

        if (CATCH_DEGENERATE_TETRAHEDRON && signd * signd < 1e-4f * 1e-4f) {
//		printf("affine dependent/degenerate\n");//
            return -1
        }
        // Points on opposite sides if expression signs are opposite
        return if (signp * signd < 0f) 1 else 0
    }

    fun closestPtPointTriangle(p: Vec3, a: Vec3, b: Vec3, c: Vec3, result: SubSimplexClosestResult): Boolean {

        result.usedVertices.reset()

        // Check if P in vertex region outside A
        val ab = b - a
        val ac = c - a
        val ap = p - a
        val d1 = ab dot ap
        val d2 = ac dot ap
        if (d1 <= 0f && d2 <= 0f) {
            result.closestPointOnSimplex put a
            result.usedVertices.usedVertexA = true
            result.setBarycentricCoordinates(1f, 0f, 0f)
            return true// a; // barycentric coordinates (1,0,0)
        }

        // Check if P in vertex region outside B
        val bp = p - b
        val d3 = ab dot bp
        val d4 = ac dot bp
        if (d3 >= 0f && d4 <= d3) {
            result.closestPointOnSimplex put b
            result.usedVertices.usedVertexB = true
            result.setBarycentricCoordinates(0f, 1f, 0f)

            return true // b; // barycentric coordinates (0,1,0)
        }
        // Check if P in edge region of AB, if so return projection of P onto AB
        val vc = d1 * d4 - d3 * d2
        if (vc <= 0f && d1 >= 0f && d3 <= 0f) {
            val v = d1 / (d1 - d3)
            result.closestPointOnSimplex put (a + v * ab)
            result.usedVertices.usedVertexA = true
            result.usedVertices.usedVertexB = true
            result.setBarycentricCoordinates(1 - v, v, 0f)
            return true
            //return a + v * ab; // barycentric coordinates (1-v,v,0)
        }

        // Check if P in vertex region outside C
        val cp = p - c
        val d5 = ab dot cp
        val d6 = ac dot cp
        if (d6 >= 0f && d5 <= d6) {
            result.closestPointOnSimplex put c
            result.usedVertices.usedVertexC = true
            result.setBarycentricCoordinates(0f, 0f, 1f)
            return true//c; // barycentric coordinates (0,0,1)
        }

        // Check if P in edge region of AC, if so return projection of P onto AC
        val vb = d5 * d2 - d1 * d6
        if (vb <= 0f && d2 >= 0f && d6 <= 0f) {
            val w = d2 / (d2 - d6)
            result.closestPointOnSimplex put (a + w * ac)
            result.usedVertices.usedVertexA = true
            result.usedVertices.usedVertexC = true
            result.setBarycentricCoordinates(1 - w, 0f, w)
            return true
            //return a + w * ac; // barycentric coordinates (1-w,0,w)
        }

        // Check if P in edge region of BC, if so return projection of P onto BC
        val va = d3 * d6 - d5 * d4
        if (va <= 0f && d4 - d3 >= 0f && d5 - d6 >= 0f) {
            val w = (d4 - d3) / ((d4 - d3) + (d5 - d6))

            result.closestPointOnSimplex put (b + w * (c - b))
            result.usedVertices.usedVertexB = true
            result.usedVertices.usedVertexC = true
            result.setBarycentricCoordinates(0f, 1 - w, w)
            return true
            // return b + w * (c - b); // barycentric coordinates (0,1-w,w)
        }

        // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
        val denom = 1f / (va + vb + vc)
        val v = vb * denom
        val w = vc * denom

        result.closestPointOnSimplex put (a + ab * v + ac * w)
        result.usedVertices.usedVertexA = true
        result.usedVertices.usedVertexB = true
        result.usedVertices.usedVertexC = true
        result.setBarycentricCoordinates(1 - v - w, v, w)

        return true
//	return a + ab * v + ac * w; // = u*a + v*b + w*c, u = va * denom = btScalar(1.0) - v - w
    }


    /** clear the simplex, remove all the vertices  */
    fun reset() {
        cachedValidClosest = false
        numVertices = 0
        needsUpdate = true
        lastW put LARGE_FLOAT
        cachedBC.reset()
    }

    /** add a vertex    */
    fun addVertex(w: Vec3, p: Vec3, q: Vec3) {
        lastW put w
        needsUpdate = true

        simplexVectorW[numVertices] put w
        simplexPointsP[numVertices] put p
        simplexPointsQ[numVertices] put q

        numVertices++
    }

    /** return/calculate the closest vertex */
    fun closest(v: Vec3): Boolean {
        val succes = updateClosestVectorAndPoints()
        v put cachedV
        return succes
    }

    fun maxVertex(): Float {
        var maxV = 0f
        repeat(numVertices) {
            val curLen2 = simplexVectorW[it].length2()
            if (maxV < curLen2) maxV = curLen2
        }
        return maxV
    }

    val fullSimplex get() = numVertices == 4

    /** return the current simplex  */
    fun getSimplex(pBuf: Array<Vec3>, qBuf: Array<Vec3>, yBuf: Array<Vec3>): Int {
        repeat(numVertices) {
            yBuf[it] put simplexVectorW[it]
            pBuf[it] put simplexPointsP[it]
            qBuf[it] put simplexPointsQ[it]
        }
        return numVertices
    }

    infix fun inSimplex(w: Vec3): Boolean {
        var found = false
        //btScalar maxV = btScalar(0.);

        //w is in the current (reduced) simplex
        repeat(numVertices) {
            val cond = when {
                USE_EQUAL_VERTEX_THRESHOLD -> simplexVectorW[it].distance2(w) <= equalVertexThreshold
                else -> simplexVectorW[it] == w
            }
            if (cond) found = true
        }

        //check in case lastW is already removed
        if (w == lastW) return true

        return found
    }

    fun backupClosest(v: Vec3) = v.put(cachedV)

    fun emptySimplex() = numVertices == 0

    fun computePoints(p1: Vec3, p2: Vec3) {
        updateClosestVectorAndPoints()
        p1 put cachedP1
        p2 put cachedP2
    }
}