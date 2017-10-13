package bullet.collision.narrowPhaseCollision

import bullet.ConvexTemplate
import bullet.DistanceTemplate
import bullet.EPSILON
import bullet.linearMath.Vec3
import bullet.linearMath.times
import kotlin.math.abs
import kotlin.math.min
import kotlin.math.sqrt

object Mpr {

    val TOLERANCE = 1E-6f
    val MAX_ITERATIONS = 1000

    class CollisionDescription {
        val firstDir = Vec3(0f, 1f, 0f)
        var maxGjkIterations = 1000
        var maximumDistanceSquared = 1e30f
        var gjkRelError2 = 1e-6f
    }

    class DistanceInfo {
        val pointOnA = Vec3()
        val pointOnB = Vec3()
        val normalBtoA = Vec3()
        var distance = 0f
    }

    class Support {
        /** Support point in minkowski sum  */
        val v = Vec3()
        /** Support point in obj1   */
        val v1 = Vec3()
        /** Support point in obj2   */
        val v2 = Vec3()
    }

    class Simplex {
        val ps = Array(4, { Support() })
        /** index of last added point   */
        var last = 0
        var size
            set(value) {
                last = value - 1
            }
            get() = last + 1

        fun point(idx: Int) = ps[idx]

        fun swap(pos1: Int, pos2: Int) {
            val supp = ps[pos1]
            ps[pos1] = ps[pos2]
            ps[pos2] = supp
        }

        operator fun set(pos: Int, a: Support) = ps.set(pos, a)

        infix fun dir(dir: Vec3) {
            val v2v1 = point(2).v - point(1).v
            val v3v1 = point(3).v - point(1).v
            dir put (v2v1 cross v3v1)
            normalize(dir)
        }


        infix fun encapsulesOrigin(dir: Vec3): Boolean {
            val dot = dir dot point(1).v
            return dot.isZero || dot > 0f
        }

        fun reachTolerance(v4: Support, dir: Vec3): Boolean {
            // find the smallest dot product of dir and {v1-v4, v2-v4, v3-v4}
            val dv1 = point(1).v dot dir
            val dv2 = point(2).v dot dir
            val dv3 = point(3).v dot dir
            val dv4 = v4.v dot dir

            var dot1 = dv4 - dv1
            val dot2 = dv4 - dv2
            val dot3 = dv4 - dv3

            dot1 = min(dot1, dot2)
            dot1 = min(dot1, dot3)

            return eq(dot1, TOLERANCE) || dot1 < TOLERANCE
        }

        fun canEncapsuleOrigin(v4: Support, dir: Vec3): Boolean {
            val dot = v4.v dot dir
            return dot.isZero || dot > 0f
        }

        infix fun expand(v4: Support) {
            val v4v0 = v4.v cross point(0).v
            var dot = point(1).v dot v4v0
            if (dot > 0f) {
                dot = point(2).v dot v4v0
                set(if (dot > 0f) 1 else 3, v4)
            } else {
                dot = point(3).v dot v4v0
                set(if (dot > 0f) 2 else 1, v4)
            }
        }
    }

    val Float.isZero get() = abs(this) < Float.EPSILON

    fun eq(_a: Float, _b: Float): Boolean {

        val ab = abs(_a - _b)
        if (abs(ab) < Float.EPSILON) return true

        val a = abs(_a)
        val b = abs(_b)
        return if (b > a) ab < Float.EPSILON * b
        else ab < Float.EPSILON * a
    }

    fun eq(a: Vec3, b: Vec3) = eq(a.x, b.x) && eq(a.y, b.y) && eq(a.z, b.z)

    fun findOrigin(a: ConvexTemplate, b: ConvexTemplate, colDesc: CollisionDescription, center: Support) {

        center.v1 put a.objectCenterInWorld
        center.v2 put b.objectCenterInWorld
        center.v put (center.v1 - center.v2) // TODO check priorities
    }

    fun normalize(d: Vec3) = d.timesAssign(1 / sqrt(d.length2()))

    fun support(a: ConvexTemplate, b: ConvexTemplate, colDesc: CollisionDescription, dir: Vec3, supp: Support) {
        val seperatingAxisInA = dir * a.worldTrans.basis
        val seperatingAxisInB = -dir * b.worldTrans.basis

        val pInA = a.getLocalSupportWithMargin(seperatingAxisInA)
        val qInB = b.getLocalSupportWithMargin(seperatingAxisInB)

        supp.v1 put (a.worldTrans * pInA) // TODO priorities
        supp.v2 put (b.worldTrans * qInB)
        supp.v put (supp.v1 - supp.v2)
    }

    fun discoverPortal(a: ConvexTemplate, b: ConvexTemplate, colDesc: CollisionDescription, portal: Simplex): Int {
        val dir = Vec3()
        val va = Vec3()
        val vb = Vec3()
        // vertex 0 is center of portal
        findOrigin(a, b, colDesc, portal.point(0))

        // vertex 0 is center of portal
        portal.size = 1

        val zero = Vec3()
        val org = zero

        if (eq(portal.point(0).v, org)) {
            /*  Portal's center lies on origin (0,0,0) => we know that objects intersect but we would need to know
                penetration info. So move center little bit...  */
            va.put(Float.EPSILON * 10f, 0f, 0f)
            portal.point(0).v += va
        }

        // vertex 1 = support in direction of origin
        dir put portal.point(0).v // TODO check btMprVec3Copy(& dir, &btMprSimplexPoint(portal, 0)->v)
        dir *= -1f
        normalize(dir)

        support(a, b, colDesc, dir, portal.point(1))

        portal.size = 2

        // test if origin isn't outside of v1
        var dot = portal.point(1).v dot dir

        if (dot.isZero || dot < 0f) return -1

        // vertex 2
        dir put (portal.point(0).v cross portal.point(1).v)
        if (dir.length2().isZero)
            return if (eq(portal.point(1).v, org)) 1    // origin lies on v1
            else 2  // origin lies on v0-v1 segment

        normalize(dir)
        support(a, b, colDesc, dir, portal.point(2))

        dot = portal.point(2).v dot dir
        if (dot.isZero || dot < 0f) return -1

        portal.size = 3

        // vertex 3 direction
        va put (portal.point(1).v - portal.point(0).v)
        vb put (portal.point(2).v - portal.point(0).v)
        dir put (va cross vb)
        normalize(dir)

        // it is better to form portal faces to be oriented "outside" origin
        dot = dir dot portal.point(0).v
        if (dot > 0f) {
            portal.swap(1, 2)
            dir *= -1f
        }

        while (portal.size < 4) {
            support(a, b, colDesc, dir, portal.point(3))

            dot = portal.point(3).v dot dir
            if (dot.isZero || dot < 0f) return -1

            var cont = 0

            // test if origin is outside (v1, v0, v3) - set v2 as v3 and continue
            va put (portal.point(1).v cross portal.point(3).v)
            dot = va dot portal.point(0).v
            if (dot < 0f && !dot.isZero) {
                portal[2] = portal.point(3)
                cont = 1
            }

            if (cont == 0) {
                // test if origin is outside (v3, v0, v2) - set v1 as v3 and continue
                va put (portal.point(3).v cross portal.point(2).v)
                dot = va dot portal.point(0).v
                if (dot < 0f && !dot.isZero) {
                    portal[1] = portal.point(3)
                    cont = 1
                }
            }

            if (cont != 0) {
                va put (portal.point(1).v - portal.point(0).v)
                vb put (portal.point(2).v - portal.point(0).v)
                dir put (va cross vb)
                normalize(dir)
            } else
                portal.size = 4
        }
        return 0
    }

    fun refinePortal(a: ConvexTemplate, b: ConvexTemplate, colDesc: CollisionDescription, portal: Simplex): Int {
        val dir = Vec3()
        val v4 = Support()

        //while (1)
        repeat(MAX_ITERATIONS) {
            // compute direction outside the portal (from v0 throught v1,v2,v3 face)
            portal dir dir

            // test if origin is inside the portal
            if (portal encapsulesOrigin dir) return 0

            // get next support point
            support(a, b, colDesc, dir, v4)

            // test if v4 can expand portal to contain origin and if portal expanding doesn't reach given tolerance
            if (!portal.canEncapsuleOrigin(v4, dir) || portal.reachTolerance(v4, dir))
                return -1

            // v1-v2-v3 triangle must be rearranged to face outside Minkowski difference (direction from v0).
            portal expand v4
        }
        return -1
    }

    fun findPos(portal: Simplex, pos: Vec3) {

        val zero = Vec3()
        val origin = zero

        val dir = Vec3()
        val b = FloatArray(4)
        val vec = Vec3()
        val p1 = Vec3()
        val p2 = Vec3()

        portal dir dir

        // use barycentric coordinates of tetrahedron to find origin
        vec put (portal.point(1).v cross portal.point(2).v)
        b[0] = vec dot portal.point(3).v

        vec put (portal.point(3).v cross portal.point(2).v)
        b[1] = vec dot portal.point(0).v

        vec put (portal.point(0).v cross portal.point(1).v)
        b[2] = vec dot portal.point(3).v

        vec put (portal.point(2).v cross portal.point(1).v)
        b[3] = vec dot portal.point(0).v

        var sum = b[0] + b[1] + b[2] + b[3]

        if (sum.isZero || sum < 0f) {
            b[0] = 0f

            vec put (portal.point(2).v cross portal.point(3).v)
            b[1] = vec dot dir
            vec put (portal.point(3).v cross portal.point(1).v)
            b[2] = vec dot dir
            vec put (portal.point(1).v cross portal.point(2).v)
            b[3] = vec dot dir

            sum = b[1] + b[2] + b[3]
        }

        val inv = 1f / sum

        p1 put origin   // TODO check copy
        p2 put origin
        for (i in 0..3) {   // TODO check copy
            vec put portal.point(i).v1
            vec *= b[i]
            p1 += vec

            vec put portal.point(i).v2
            vec *= b[i]
            p2 += vec
        }
        p1 *= inv
        p2 *= inv
        pos put p2
    }

    infix fun Vec3.dist2(b: Vec3): Float {
        val abX = x - b.x
        val abY = y - b.y
        val abZ = z - b.z
        return abX * abX + abY * abY + abZ * abZ
    }

    fun vec3PointSegmentDist2(p: Vec3, x0: Vec3, b: Vec3, witness: Vec3?): Float {
        /*  The computation comes from solving equation of segment:
                S(t) = x0 + t.d
            where:
                - x0 is initial point of segment
                - d is direction of segment from x0 (|d| > 0)
                - t belongs to <0, 1> interval
            Than, distance from a segment to some point P can be expressed:
                D(t) = |x0 + t.d - P|^2
            which is distance from any point on segment. Minimization of this function brings distance from P to segment.
            Minimization of D(t) leads to simple quadratic equation that's solving is straightforward.
            Bonus of this method is witness point for free. */

        val d = Vec3()
        val a = Vec3()

        // direction of segment
        d put (b - x0)

        // precompute vector from P to x0
        a put (x0 - p)

        var t = -1f * (a dot d)
        t /= d.length2()

        return when {
            t < 0f || t.isZero -> {
                witness?.put(x0)
                x0 dist2 p
            }
            t > 1f || eq(t, 1f) -> {
                witness?.put(b)
                b dist2 p
            }
            else ->
                if (witness != null) {
                    witness put d
                    witness *= t
                    witness += x0
                    witness dist2 p
                } else {    // recycling variables
                    d *= t
                    d += a
                    d.length2()
                }
        }
    }

    fun vec3PointTriDist2(pt: Vec3, x0: Vec3, b: Vec3, c: Vec3, witness: Vec3?): Float {
        /*  Computation comes from analytic expression for triangle (x0, B, C)
                T(s, t) = x0 + s.d1 + t.d2
             where
                d1 = B - x0
                d2 = C - x0
            Then equation for distance is:
                D(s, t) = | T(s, t) - P |^2
            This leads to minimization of quadratic function of two variables.
            The solution from is taken only if s is between 0 and 1, t is between 0 and 1 and t + s < 1,
            otherwise distance from segment is computed.    */
        val d1 = Vec3()
        val d2 = Vec3()
        val a = Vec3()
        val s: Float
        var t = 0f
        var dist: Float
        val witness2 = Vec3()

        d1 put (b - x0)
        d2 put (c - x0)
        a put (x0 - pt)

        val u = a dot a
        val v = d1 dot d1
        val w = d2 dot d2
        val p = a dot d1
        val q = a dot d2
        val r = d1 dot d2

        val div = w * v - r * r
        if (div.isZero) {
            s = -1f
        } else {
            s = (q * r - w * p) / div
            t = (-s * r - q) / w
        }

        if ((s.isZero || s > 0f) && (eq(s, 1f) || s < 1f) && (t.isZero || t > 0f) && (eq(t, 1f) || t < 1f)
                && (eq(t + s, 1f) || t + s < 1f))

            dist = if (witness != null) {
                d1 *= s
                d2 *= t
                witness put x0
                witness += d1
                witness += d2
                witness dist2 pt
            } else s * s * v + t * t * w + 2f * s * t * r + 2f * s * p + 2f * t * q + u
        else {
            dist = vec3PointSegmentDist2(pt, x0, b, witness)

            var dist2 = vec3PointSegmentDist2(pt, x0, c, witness2)
            if (dist2 < dist) {
                dist = dist2
                witness?.put(witness2)
            }
            dist2 = vec3PointSegmentDist2(pt, b, c, witness2)
            if (dist2 < dist) {
                dist = dist2
                witness?.put(witness2)
            }
        }
        return dist
    }

    /** @return depth   */
    fun findPenetr(a: ConvexTemplate, b: ConvexTemplate, colDesc: CollisionDescription, portal: Simplex, depth: Float, pdir: Vec3,
                   pos: Vec3): Float {
        val dir = Vec3()
        val v4 = Support()
        var iterations = 1

        val zero = Vec3()
        val origin = zero
        var depth = depth

        for (i in 0 until MAX_ITERATIONS) {        //while (1)
            // compute portal direction and obtain next support point
            portal dir dir

            support(a, b, colDesc, dir, v4)

            // reached tolerance -> find penetration info
            if (portal.reachTolerance(v4, dir) || iterations == MAX_ITERATIONS) {
                depth = vec3PointTriDist2(origin, portal.point(1).v, portal.point(2).v, portal.point(3).v, pdir)
                depth = sqrt(depth)

                if (pdir.x.isZero && pdir.y.isZero && pdir.z.isZero)
                    pdir put dir
                normalize(pdir)

                // barycentric coordinates:
                findPos(portal, pos)

                return depth
            }

            portal expand v4

            iterations++
        }
        return depth
    }

    /** @return depth   */
    fun findPenetrTouch(portal: Simplex, dir: Vec3, pos: Vec3): Float {
        //  Touching contact on portal's v1 - so depth is zero and direction is unimportant and pos can be guessed
        val zero = Vec3()
        val origin = zero

        dir put origin
        pos put portal.point(1).v2
        return 0f
    }

    /** @return depth   */
    fun findPenetrSegment(portal: Simplex, dir: Vec3, pos: Vec3): Float {
        // Origin lies on v0-v1 segment. Depth is distance to v1, direction also and position must be computed
        pos put portal.point(1).v2

        dir put portal.point(1).v
        val depth = sqrt(dir.length2())
        normalize(dir)
        return depth
    }


    fun penetration(a: ConvexTemplate, b: ConvexTemplate, colDesc: CollisionDescription, depthOut: FloatArray, dirOut: Vec3,
                    posOut: Vec3): Int {

        val portal = Simplex()

        // Phase 1: Portal discovery
        var result = discoverPortal(a, b, colDesc, portal)

        //sepAxis[pairIndex] = *pdir;//or -dir?

        when (result) {
            0 -> {  // Phase 2: Portal refinement
                result = refinePortal(a, b, colDesc, portal)
                if (result < 0) return -1
                // Phase 3. Penetration info
                depthOut[0] = findPenetr(a, b, colDesc, portal, depthOut[0], dirOut, posOut)
            }
            1 -> {  // Touching contact on portal's v1.
                depthOut[0] = findPenetrTouch(portal, dirOut, posOut)
                result = 0
            }
            2 -> {
                depthOut[0] = findPenetrSegment(portal, dirOut, posOut)
                result = 0
            }
            else -> result = -1
        }
        return result
    }


    fun computeMprPenetration(a: ConvexTemplate, b: ConvexTemplate, colDesc: CollisionDescription, distInfo: DistanceTemplate): Int {
        val dir = Vec3()
        val pos = Vec3()
        val depth = FloatArray(1)

        val res = penetration(a, b, colDesc, depth, dir, pos)
        return if (res == 0) with(distInfo) {
            distance = -depth[0]
            pointOnB put pos
            normalBtoA put -dir
            pointOnA put (pos - distInfo.distance * dir)
            0
        } else -1
    }
}