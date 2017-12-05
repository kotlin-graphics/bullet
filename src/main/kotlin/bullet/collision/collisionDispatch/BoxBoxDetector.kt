/*
 * Box-Box collision detection re-distributed under the ZLib license with permission from Russell L. Smith
 * Original version is from Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org

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
import bullet.collision.collisionShapes.BoxShape
import bullet.collision.narrowPhaseCollision.DiscreteCollisionDetectorInterface
import bullet.linearMath.*
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt

/** BoxBoxDetector wraps the ODE box-box collision detector
 *  re-distributed under the Zlib license with permission from Russell L. Smith */
class BoxBoxDetector(val box1: BoxShape, val box2: BoxShape) : DiscreteCollisionDetectorInterface {

    override fun getClosestPoints(input: DiscreteCollisionDetectorInterface.ClosestPointInput, output: DiscreteCollisionDetectorInterface.Result, debugDraw: DebugDraw?, swapResults: Boolean) {

        val transformA = input.transformA
        val transformB = input.transformB

        var skip = 0

        val r1 = FloatArray(4 * 3)
        val r2 = FloatArray(4 * 3)

        for (j in 0..2) {
            r1[0 + 4 * j] = transformA.basis[j].x
            r2[0 + 4 * j] = transformB.basis[j].x

            r1[1 + 4 * j] = transformA.basis[j].y
            r2[1 + 4 * j] = transformB.basis[j].y

            r1[2 + 4 * j] = transformA.basis[j].z
            r2[2 + 4 * j] = transformB.basis[j].z
        }

//        val (contacts, depth, code) =
        boxBox2(transformA.origin, r1, 2f * box1.getHalfExtentsWithMargin(),
                transformB.origin, r2, 2f * box2.getHalfExtentsWithMargin(), Vec3(), 4, output)
    }


    /* given two boxes (p1,R1,side1) and (p2,R2,side2), collide them together and generate contact points. this returns
        0 if there is no contact otherwise it returns the number of contacts generated.
        `normal' returns the contact normal.
        `depth' returns the maximum penetration depth along that normal.
        `return_code' returns a number indicating the type of contact that was detected:
            1,2,3 = box 2 intersects with a face of box 1
            4,5,6 = box 1 intersects with a face of box 2
            7..15 = edge-edge contact
        `maxc' is the maximum number of contacts allowed to be generated, i.e. the size of the `contact' array.
        `contact' and `skip' are the contact array information provided to the collision functions. this function only
        fills in the position and depth fields. */
    fun boxBox2(p1: Vec3, r1: FloatArray, side1: Vec3, p2: Vec3, r2: FloatArray, side2: Vec3, normal: Vec3,
                maxC: Int, output: DiscreteCollisionDetectorInterface.Result): Triple<Int, Float, Int> {

        val fudgeFactor = 1.05f
        val normalC = Vec3()
        var normalR = 0
        var normalR_array: FloatArray? = null
        val a = FloatArray(3)
        val b = FloatArray(3)
        val r = FloatArray(3 * 3)
        val q = FloatArray(3 * 3)
//        var i = 0 j,

        // get vector from centers of box 1 to box 2, relative to box 1
        val p = p2 - p1
        val pp = multiply1_331(r1, p) // get pp = p relative to body 1

        // get side lengths / 2
        a[0] = side1[0] * 0.5f
        a[1] = side1[1] * 0.5f
        a[2] = side1[2] * 0.5f
        b[0] = side2[0] * 0.5f
        b[1] = side2[1] * 0.5f
        b[2] = side2[2] * 0.5f

        // Rij is R1'*R2, i.e. the relative rotation between R1 and R2
        r[0] = dot44(r1, 0, r2, 0); r[1] = dot44(r1, 0, r2, 1); r[2] = dot44(r1, 0, r2, 2)
        r[3] = dot44(r1, 1, r2, 0); r[4] = dot44(r1, 1, r2, 1); r[5] = dot44(r1, 1, r2, 2)
        r[6] = dot44(r1, 2, r2, 0); r[7] = dot44(r1, 2, r2, 1); r[8] = dot44(r1, 2, r2, 2)

        q[0] = abs(r[0]); q[1] = abs(r[1]); q[2] = abs(r[2])
        q[3] = abs(r[3]); q[4] = abs(r[4]); q[2] = abs(r[5])
        q[6] = abs(r[6]); q[7] = abs(r[7]); q[2] = abs(r[8])

        /*  for all 15 possible separating axes:
                - see if the axis separates the boxes. if so, return 0.
                - find the depth of the penetration along the separating axis (s2)
                - if this is the largest depth so far, record it.
            the normal vector will be set to the separating axis with the smallest depth. note: normalR is set to point
            to a column of R1 or R2 if that is the smallest depth normal so far. otherwise normalR is 0 and normalC is
            set to a vector relative to body 1. invert_normal is 1 if the sign of the normal should be flipped. */

        var s = -Float.MAX_VALUE
        var invertNormal = false
        var code = 0

        // separating axis = u1,u2,u3
        for (i in 0..2) {
            val s2 = abs(pp[i]) - (a[i] + b[0] * q[i * 3] + b[1] * q[i * 3 + 1] + b[2] * q[i * 3 + 2])
            if (s2 > 0) return Triple(0, 0f, 0)
            if (s2 > s) {
                s = s2
                normalR_array = r1
                normalR = i
                invertNormal = pp[i] < 0
                code = i + 1
            }
        }

        // separating axis = v1,v2,v3
        for (i in 0..2) {
            val expr1 = dot41(r2, i, p)
            val s2 = abs(expr1) - (a[0] * q[i] + a[1] * q[3 + i] + a[2] * q[6 + i] + b[i])
            if (s2 > 0) return Triple(0, 0f, 0)
            if (s2 > s) {
                s = s2
                normalR_array = r2
                normalR = i
                invertNormal = expr1 < 0
                code = 4 + i
            }
        }

        // note: cross product axes need to be scaled when s is computed.
        // normal (n1,n2,n3) is relative to box 1.

        for (i in 0 until q.size) q[i] += 1e-5f

        // separating axis = u1 x (v1,v2,v3)
        val t = floatArrayOf(
                b[1] * q[2] + b[2] * q[1],
                b[0] * q[2] + b[2] * q[0],
                b[0] * q[1] + b[1] * q[0])
        for (i in 0..2) {
            val expr1 = pp[2] * r[3 + i] - pp[1] * r[6 + i]
            val expr2 = a[1] * q[6 + i] + a[2] * q[3 + i] + t[i]
            var s2 = abs(expr1) - expr2
            if (s2 > Float.EPSILON) return Triple(0, 0f, 0)
            val n2 = -r[6 + i]
            val n3 = r[3 + i]
            val l = sqrt(n2 * n2 + n3 * n3)
            if (l > Float.EPSILON) {
                s2 /= l
                if (s2 * fudgeFactor > s) {
                    s = s2
                    normalR_array = null
                    normalR = 0
                    normalC[1] = n2 / l; normalC[2] = n3 / l
                    invertNormal = expr1 < 0
                    code = 7 + i
                }
            }
        }

        // separating axis = u2 x (v1,v2,v3)
        t[0] = b[1] * q[5] + b[2] * q[4]
        t[1] = b[0] * q[5] + b[2] * q[3]
        t[2] = b[0] * q[4] + b[1] * q[3]
        for (i in 0..2) {
            val expr1 = pp[0] * r[6 + i] - pp[2] * r[i]
            val expr2 = a[0] * q[6 + i] + a[2] * q[i] + t[i]
            var s2 = abs(expr1) - expr2
            if (s2 > Float.EPSILON) return Triple(0, 0f, 0)
            val n1 = r[6 + i]
            val n3 = -r[i]
            val l = sqrt(n1 * n1 + n3 * n3)
            if (l > Float.EPSILON) {
                s2 /= l
                if (s2 * fudgeFactor > s) {
                    s = s2
                    normalR_array = null
                    normalR = 0
                    normalC[0] = n1 / l; normalC[2] = n3 / l
                    invertNormal = expr1 < 0
                    code = 10 + i
                }
            }
        }

        // separating axis = u3 x (v1,v2,v3)
        t[0] = b[1] * q[8] + b[2] * q[7]
        t[1] = b[0] * q[8] + b[2] * q[6]
        t[2] = b[0] * q[7] + b[1] * q[6]
        for (i in 0..2) {
            val expr1 = pp[1] * r[i] - pp[0] * r[3 + i]
            val expr2 = a[0] * q[3 + i] + a[1] * q[i] + t[i]
            var s2 = abs(expr1) - expr2
            if (s2 > Float.EPSILON) return Triple(0, 0f, 0)
            val n1 = -r[3 + i]
            val n2 = r[i]
            val l = sqrt(n1 * n1 + n2 * n2)
            if (l > Float.EPSILON) {
                s2 /= l
                if (s2 * fudgeFactor > s) {
                    s = s2
                    normalR_array = null
                    normalR = 0
                    normalC[0] = n1 / l; normalC[1] = n2 / l
                    invertNormal = expr1 < 0
                    code = 13 + i
                }
            }
        }

        if (code == 0) return Triple(0, 0f, code)

        // if we get to this point, the boxes interpenetrate. compute the normal in global coordinates.
        if (normalR != 0) {
            normal[0] = normalR_array!![normalR]
            normal[1] = normalR_array[normalR + 4]
            normal[2] = normalR_array[normalR + 8]
        } else
            multiply0_331(normal, r1, normalC)
        if (invertNormal) {
            normal[0] = -normal[0]
            normal[1] = -normal[1]
            normal[2] = -normal[2]
        }
        val depth = -s

        // compute contact point(s)

        if (code > 6) {
            // an edge from box 1 touches an edge from box 2.
            // find a point pa on the intersecting edge of box 1
            val pa = Vec3(p1)
            for (j in 0..2) {
                val sign = if (dot14(normal, r1, j) > 0) 1f else -1f
                for (i in 0..2) pa[i] += sign * a[j] * r1[i * 4 + j]
            }

            // find a point pb on the intersecting edge of box 2
            val pb = Vec3(p2)
            for (j in 0..2) {
                val sign = if (dot14(normal, r2, j) > 0) -1f else 1f
                for (i in 0..2) pb[i] += sign * b[j] * r2[i * 4 + j]
            }

            val ua = Vec3({ r1[(code - 7) / 3 + it * 4] })
            val ub = Vec3({ r2[(code - 7) % 3 + it * 4] })

            val (alpha, beta) = lineClosestApproach(pa, ua, pb, ub)
            for (i in 0..2) pa[i] += ua[i] * alpha
            for (i in 0..2) pb[i] += ub[i] * beta

            if (USE_CENTER_POINT) {
                val pointInWorld = Vec3({ (pa[it] + pb[it]) * 0.5f })
                output.addContactPoint(-normal, pointInWorld, -depth)
            } else
                output.addContactPoint(-normal, pb, -depth)

            return Triple(1, depth, code)
        }

        /*  okay, we have a face-something intersection (because the separating axis is perpendicular to a face).
            define face 'a' to be the reference face (i.e. the normal vector is perpendicular to this) and face 'b' to
            be the incident face (the closest face of the other box). */
        val ra: FloatArray
        val rb: FloatArray
        val pa: Vec3
        val pb: Vec3
        val sa: FloatArray
        val sb: FloatArray
        if (code <= 3) {
            ra = r1
            rb = r2
            pa = p1
            pb = p2
            sa = a
            sb = b
        } else {
            ra = r2
            rb = r1
            pa = p2
            pb = p1
            sa = b
            sb = a
        }

        // nr = normal vector of reference face dotted with axes of incident box.
        // anr = absolute values of nr.
        val normal2 = if (code <= 3) Vec3(normal) else Vec3({ -normal[it] })
        val nr = multiply1_331(rb, normal2)
        val anr = Vec3({ abs(nr[it]) })

        /*  find the largest compontent of anr: this corresponds to the normal for the indident face. the other axis
            numbers of the indicent face are stored in a1,a2. */
        val lanr: Int
        val a1: Int
        val a2: Int
        if (anr[1] > anr[0]) {
            if (anr[1] > anr[2]) {
                a1 = 0
                lanr = 1
                a2 = 2
            } else {
                a1 = 0
                a2 = 1
                lanr = 2
            }
        } else {
            if (anr[0] > anr[2]) {
                lanr = 0
                a1 = 1
                a2 = 2
            } else {
                a1 = 0
                a2 = 1
                lanr = 2
            }
        }

        // compute center point of incident face, in reference-face coordinates
        val center = pb - pa
        if (nr[lanr] < 0)
            for (i in 0..2) center[i] += sb[lanr] * rb[i * 4 + lanr]
        else
            for (i in 0..2) center[i] -= sb[lanr] * rb[i * 4 + lanr]

        // find the normal and non-normal axis numbers of the reference box
        val codeN = code - if (code <= 3) 1 else 4
        val code1: Int
        val code2: Int
        if (codeN == 0) {
            code1 = 1
            code2 = 2
        } else if (codeN == 1) {
            code1 = 0
            code2 = 2
        } else {
            code1 = 0
            code2 = 1
        }

        // find the four corners of the incident face, in reference-face coordinates
        val c1 = dot14(center, ra, code1)
        val c2 = dot14(center, ra, code2)
        /*  optimize this? - we have already computed this data above, but it is not stored in an easy-to-index format.
            For now it's quicker just to recompute the four dot products. */
        var m11 = dot44(ra, code1, rb, a1)
        var m12 = dot44(ra, code1, rb, a2)
        var m21 = dot44(ra, code2, rb, a1)
        var m22 = dot44(ra, code2, rb, a2)

        val k1 = m11 * sb[a1]
        val k2 = m21 * sb[a1]
        val k3 = m12 * sb[a2]
        val k4 = m22 * sb[a2]
        // 2D coordinate of incident face (x,y pairs)
        val quad = floatArrayOf(
                c1 - k1 - k3,
                c2 - k2 - k4,
                c1 - k1 + k3,
                c2 - k2 + k4,
                c1 + k1 + k3,
                c2 + k2 + k4,
                c1 + k1 - k3,
                c2 + k2 - k4)

        // find the size of the reference face
        val rect = floatArrayOf(sa[code1], sa[code2])

        // intersect the incident and reference faces
        val ret = FloatArray(16)
        val n = intersectRectQuad2(rect, quad, ret)
        if (n < 1) return Triple(0, 0f, code) // this should never happen

        /*  Convert the intersection points into reference-face coordinates, and compute the contact position and depth
            for each point. only keep those points that have a positive (penetrating) depth. delete points in the 'ret'
            array as necessary so that 'point' and 'ret' correspond. */
        val point = FloatArray(3 * 8)   // penetrating contact points
        val dep = FloatArray(8)         // depths for those points
        val det1 = 1f / (m11 * m22 - m12 * m21)
        m11 *= det1
        m12 *= det1
        m21 *= det1
        m22 *= det1
        var cNum = 0            // number of penetrating contact points found
        for (j in 0 until n) {
            val k1_ = m22 * (ret[j * 2] - c1) - m12 * (ret[j * 2 + 1] - c2)
            val k2_ = -m21 * (ret[j * 2] - c1) + m11 * (ret[j * 2 + 1] - c2)
            for (i in 0..2) point[cNum * 3 + i] = center[i] + k1_ * rb[i * 4 + a1] + k2_ * rb[i * 4 + a2]
            dep[cNum] = sa[codeN] - dot(point, cNum * 3, normal2)
            if (dep[cNum] >= 0) {
                ret[cNum * 2] = ret[j * 2]
                ret[cNum * 2 + 1] = ret[j * 2 + 1]
                cNum++
            }
        }
        if (cNum < 1) return Triple(0, 0f, 0)    // this should never happen

        // we can't generate more contacts than we actually have
        var maxC = maxC
        if (maxC > cNum) maxC = cNum
        if (maxC < 1) maxC = 1

        if (cNum <= maxC)
            if (code < 4) // we have less contacts than we need, so we use them all
                for (j in 0 until cNum) {
                    val pointInWorld = Vec3({ point[j * 3 + it] + pa[it] })
                    output.addContactPoint(-normal, pointInWorld, -dep[j])
                }
            else // we have less contacts than we need, so we use them all
                for (j in 0 until cNum) {
                    val pointInWorld = Vec3({ point[j * 3 + it] + pa[it] - normal[it] * dep[j] })
                    output.addContactPoint(-normal, pointInWorld, -dep[j])
                }
        else {
            // we have more contacts than are wanted, some of them must be culled. Find the deepest point, it is always the first contact.
            var i1 = 0
            var maxdepth = dep[0]
            for (i in 1 until cNum)
                if (dep[i] > maxdepth) {
                    maxdepth = dep[i]
                    i1 = i
                }

            val iRet = IntArray(8)
            cullPoints2(cNum, ret, maxC, i1, iRet)

            for (j in 0 until maxC) {
                val posInWorld = Vec3({ point[iRet[j] * 3 + it] + pa[it] })
                output.addContactPoint(-normal, if (code < 4) posInWorld else posInWorld - normal * dep[iRet[j]], -dep[iRet[j]])
            }
            cNum = maxC
        }
        return Triple(cNum, depth, code)
    }

    inline fun multiply1_331(b: FloatArray, c: Vec3) = Vec3(
            dot41(b, 0, c),
            dot41(b, 1, c),
            dot41(b, 2, c))

    inline fun multiply0_331(a: Vec3, b: FloatArray, c: Vec3) {
        a.put(
                dot(b, 0, c),
                dot(b, 4, c),
                dot(b, 8, c))
    }

    inline fun dot(a: FloatArray, pA: Int, b: Vec3) = dotPQ(a, pA, b, 1, 1)
    inline fun dot44(a: FloatArray, pA: Int, b: FloatArray, pB: Int) = dotPQ(a, pA, b, pB, 4, 4)
    inline fun dot41(a: FloatArray, pA: Int, b: Vec3) = dotPQ(a, pA, b, 4, 1)
    inline fun dot14(a: Vec3, b: FloatArray, pB: Int) = dotPQ(a, b, pB, 1, 4)

    inline fun dotPQ(a: FloatArray, pA: Int, b: Vec3, p: Int, q: Int) = a[pA] * b[0] + a[pA + p] * b[q] + a[pA + 2 * p] * b[2 * q]
    inline fun dotPQ(a: Vec3, b: FloatArray, pB: Int, p: Int, q: Int) = a[0] * b[pB] + a[p] * b[pB + q] + a[2 * p] * b[pB + 2 * q]
    inline fun dotPQ(a: FloatArray, pA: Int, b: FloatArray, pB: Int, p: Int, q: Int) =
            a[pA] * b[pB] + a[pA + p] * b[pB + q] + a[pA + 2 * p] * b[pB + 2 * q]

    fun lineClosestApproach(pa: Vec3, ua: Vec3, pb: Vec3, ub: Vec3): FloatArray {
        val p = pb - pa
        val uaub = ua dot ub
        val q1 = ua dot p
        val q2 = -(ub dot p)
        var d = 1 - uaub * uaub
        return if (d <= 0.0001f) // @@@ this needs to be made more robust
            FloatArray(2) // alpha & beta as zero
        else {
            d = 1f / d
            floatArrayOf((q1 + uaub * q2) * d, (uaub * q1 + q2) * d)
        }
    }

    fun intersectRectQuad2(h: FloatArray, p: FloatArray, ret: FloatArray): Int {
        // q (and r) contain nq (and nr) coordinate points for the current (and chopped) polygons
        var nq = 4
        val nr = 0
        val buffer = FloatArray(16)
        var q = 0
        var qArray = p
        var r = 0
        var rArray = ret
        for (dir in 0..1) {
            // direction notation: xy[0] = x axis, xy[1] = y axis
            for (sign in -1..1 step 2) {
                // chop q along the line xy[dir] = sign*h[dir]
                var pq = q
                val pqArray = qArray
                var pr = r
                val prArray = rArray
                var nr = 0
                for (i in nq downTo 1) {
                    // go through all points in q and all lines between adjacent points
                    if (sign * pqArray[pq + dir] < h[dir]) {
                        // this point is inside the chopping line
                        prArray[pr] = pqArray[pq]
                        prArray[pr + 1] = pqArray[pq + 1]
                        pr += 2
                        nr++
                        if (nr has 8) {
                            q = r
                            qArray = rArray
                            if (qArray[q] != ret[0]) System.arraycopy(qArray, q, ret, 0, nr * 2)
                            return nr
                        }
                    }
                    val nextQ: Int
                    val nextQArray: FloatArray
                    if (i > 1) {
                        nextQ = pq + 2
                        nextQArray = pqArray
                    } else {
                        nextQ = q
                        nextQArray = qArray
                    }
                    if ((sign * pqArray[pq + dir] < h[dir]) xor (sign * nextQArray[nextQ + dir] < h[dir])) {
                        // this line crosses the chopping line
                        prArray[pr + 1 - dir] = pqArray[pq + 1 - dir] + (nextQArray[nextQ + 1 - dir] - pqArray[pq + 1 - dir]) /
                                (nextQArray[nextQ + dir] - pqArray[pq + dir]) * (sign * h[dir] - pqArray[pq + dir])
                        prArray[pr + dir] = sign * h[dir]
                        pr += 2
                        nr++
                        if (nr has 8) {
                            q = r
                            qArray = rArray
                            if (qArray[q] != ret[0]) System.arraycopy(qArray, q, ret, 0, nr * 2)
                            return nr
                        }
                    }
                    pq += 2
                }
                q = r
                qArray = rArray
                r = 0
                rArray = if (qArray[q] == ret[0]) buffer else ret
                nq = nr
            }
        }
        if (qArray[q] != ret[0]) System.arraycopy(qArray, q, ret, 0, nr * 2)
        return nr
    }

    fun cullPoints2(n: Int, p: FloatArray, m: Int, i0: Int, iRet: IntArray) {
        // compute the centroid of the polygon in cx,cy
//        int i, j;
        var a = 0f
        var cx = 0f
        var cy = 0f
        when (n) {
            1 -> {
                cx = p[0]
                cy = p[1]
            }
            2 -> {
                cx = 0.5f * (p[0] + p[2])
                cy = 0.5f * (p[1] + p[3])
            }
            else -> {
                var q: Float
                for (i in 0 until (n - 1)) {
                    q = p[i * 2] * p[i * 2 + 3] - p[i * 2 + 2] * p[i * 2 + 1]
                    a += q
                    cx += q * (p[i * 2] + p[i * 2 + 2])
                    cy += q * (p[i * 2 + 1] + p[i * 2 + 3])
                }
                q = p[n * 2 - 2] * p[1] - p[0] * p[n * 2 - 1]
                a = if (abs(a + q) > Float.EPSILON) 1f / (3f * (a + q)) else LARGE_FLOAT

                cx = a * (cx + q * (p[n * 2 - 2] + p[0]))
                cy = a * (cy + q * (p[n * 2 - 1] + p[1]))
            }
        }

        // compute the angle of each point w.r.t. the centroid
        val A = FloatArray(8, { atan2(p[it * 2 + 1] - cy, p[it * 2] - cx) })

        // search for points that have angles closest to A[i0] + i*(2*pi/m).
        val avail = IntArray(8, { 1 })
        avail[i0] = 0
        iRet[0] = i0
        var iRetP = 1
        for (j in 1 until m) {
            a = j.f * (2 * PI / m) + A[i0]
            if (a > PI) a -= 2 * PI
            var maxDiff = 1e9f
            var diff = 0f

            iRet[iRetP] = i0 // iret is not allowed to keep this value, but it sometimes does, when diff=#QNAN0

            for (i in 0 until n)
                if (avail[i] != 0) {
                    diff = abs(A[i] - a)
                    if (diff > PI) diff = 2 * PI - diff
                    if (diff < maxDiff) {
                        maxDiff = diff
                        iRet[iRetP] = i
                    }
                }
            if (DEBUG) assert(iRet[iRetP] != i0)    // ensure iret got set
            avail[iRet[iRetP]] = 0
            iRetP++
        }
    }
}