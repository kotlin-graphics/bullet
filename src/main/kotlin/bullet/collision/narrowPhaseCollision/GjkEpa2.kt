package bullet.collision.narrowPhaseCollision

import bullet.collision.collisionShapes.ConvexShape
import bullet.has
import bullet.linearMath.Mat3
import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import bullet.linearMath.max
import kotlin.math.abs
import kotlin.math.sqrt

object GjkEpaSolver2 {

    class Results {
        enum class Status {
            Invalid,
            /* Shapes doesnt penetrate  */
            Separated,
            /* Shapes are penetrating   */
            Penetrating,
            /* GJK phase fail, no big issue, shapes are probably just 'touching'	*/
            GJK_Failed,
            /* EPA phase fail, bigger problem, need to save parameters, and debug	*/
            EPA_Failed
        }

        var status = Status.Invalid
        val witnesses = Array(2, { Vec3() })
        val normal = Vec3()
        var distance = 0f
    }

    // Config

    /* GJK	*/
    val GJK_MAX_ITERATIONS = 128

    val GJK_ACCURACY = 0.0001f
    val GJK_MIN_DISTANCE = 0.0001f
    val GJK_DUPLICATED_EPS = 0.0001f


    val GJK_SIMPLEX2_EPS = 0f
    val GJK_SIMPLEX3_EPS = 0f
    val GJK_SIMPLEX4_EPS = 0f

    /* EPA	*/
    val EPA_MAX_VERTICES = 128
    val EPA_MAX_ITERATIONS = 255

    val EPA_ACCURACY = 0.0001f
    val EPA_PLANE_EPS = 0.00001f
    val EPA_INSIDE_EPS = 0.01f

    val EPA_FALLBACK = 10 * EPA_ACCURACY
    val EPA_MAX_FACES = EPA_MAX_VERTICES * 2

    class MinkowskiDiff {
        lateinit var shapes: Array<ConvexShape>
        val toShape1 = Mat3()
        val toShape0 = Transform()
        var ls: ((Vec3) -> Vec3)? = null
        private var margin = false

        fun support0(d: Vec3) = when {
            margin -> shapes[0].localGetSupportVertexNonVirtual(d)
            else -> shapes[0].localGetSupportVertexWithoutMarginNonVirtual(d)
        }

        fun support1(d: Vec3) = toShape0 * when {
            margin -> shapes[1].localGetSupportVertexNonVirtual(toShape1 * d)
            else -> shapes[1].localGetSupportVertexWithoutMarginNonVirtual(toShape1 * d)
        }

        fun support(d: Vec3) = support0(d) - support1(-d)
        fun support(d: Vec3, index: Int) = when {
            index != 0 -> support1(d)
            else -> support0(d)
        }
    }

    class GJK {
        /* Types		*/
        class SV {
            val d = Vec3()
            val w = Vec3()
        }

        class Simplex {
            val c = Array(4, { SV() })
            val p = FloatArray(4)
            var rank = 0
        }

        enum class Status { Valid, Inside, Failed }

        /* Fields		*/
        lateinit var shape: MinkowskiDiff
        val ray = Vec3()
        var distance = 0f
        val simplices = Array(2, { Simplex() })
        val store = Array(4, { SV() })
        val free = Array(4, { SV() })
        var nfree = 0
        var current = 0
        var simplex = Simplex()
        var status = Status.Failed

        /* Methods		*/
        fun initialize() {
            ray put 0f
            nfree = 0
            status = Status.Failed
            current = 0
            distance = 0f
        }

        fun evaluate(shapearg: MinkowskiDiff, guess: Vec3): Status {
            var iterations = 0
            var alpha = 0f
            val lastw = Array(4, { Vec3() })
            var clastw = 0
            /* Initialize solver		*/
            free[0] = store[0]
            free[1] = store[1]
            free[2] = store[2]
            free[3] = store[3]
            nfree = 4
            current = 0
            status = Status.Valid
            shape = shapearg
            distance = 0f
            /* Initialize simplex		*/
            simplices[0].rank = 0
            ray put guess
            val sqrl = ray.length2()
            appendVertice(simplices[0], if (sqrl > 0) -ray else Vec3(1f, 0f, 0f))
            simplices[0].p[0] = 1f
            ray put simplices[0].c[0].w
            var sqdist = sqrl
            lastw[0] = Vec3(ray)
            lastw[1] = Vec3(ray)
            lastw[2] = Vec3(ray)
            lastw[3] = Vec3(ray)
            /* Loop						*/
            do {
                val next = 1 - current
                val cs = simplices[current]
                val ns = simplices[next]
                /* Check zero							*/
                val rl = ray.length()
                if (rl < GJK_MIN_DISTANCE) {/* Touching or inside				*/
                    status = Status.Inside
                    break
                }
                /* Append new vertice in -'v' direction	*/
                appendVertice(cs, -ray)
                val w = cs.c[cs.rank - 1].w
                var found = false
                for (i in 0..3)
                    if ((w - lastw[i]).length2() < GJK_DUPLICATED_EPS) {
                        found = true
                        break
                    }
                if (found) {/* Return old simplex				*/
                    removeVertice(simplices[current])
                    break
                } else {/* Update lastw					*/
                    clastw = (clastw + 1) and 3
                    lastw[clastw] = w
                }
                /* Check for termination				*/
                val omega = (ray dot w) / rl
                alpha = omega max alpha
                if (rl - alpha - GJK_ACCURACY * rl <= 0) {/* Return old simplex				*/
                    removeVertice(simplices[current])
                    break
                }
                /* Reduce simplex						*/
                val weights = FloatArray(4)
                val p = intArrayOf(0)
                when (cs.rank) {
                    2 -> sqdist = projectOrigin(cs.c[0].w, cs.c[1].w, weights, p)
                    3 -> sqdist = projectOrigin(cs.c[0].w, cs.c[1].w, cs.c[2].w, weights, p)
                    4 -> sqdist = projectOrigin(cs.c[0].w, cs.c[1].w, cs.c[2].w, cs.c[3].w, weights, p)
                }
                val mask = p[0]
                if (sqdist >= 0) {/* Valid	*/
                    ns.rank = 0
                    ray put 0f
                    current = next
                    for (i in 0 until cs.rank)
                        if (mask has (1 shl i)) {
                            ns.c[ns.rank] = cs.c[i]
                            ns.p[ns.rank++] = weights[i]
                            ray += cs.c[i].w * weights[i]
                        } else
                            free[nfree++] = cs.c[i]
                    if (mask == 15) status = Status.Inside
                } else {/* Return old simplex				*/
                    removeVertice(simplices[current])
                    break
                }
                status = if (++iterations < GJK_MAX_ITERATIONS) status else Status.Failed
            } while (status == Status.Valid)
            simplex = simplices[current]
            when (status) {
                Status.Valid -> distance = ray.length()
                Status.Inside -> distance = 0f
                else -> Unit
            }
            return (status)
        }

        fun encloseOrigin(): Boolean {
            when (simplex.rank) {
                1 -> {
                    for (i in 0..2) {
                        val axis = Vec3()
                        axis[i] = 1f
                        appendVertice(simplex, axis)
                        if (encloseOrigin()) return true
                        removeVertice(simplex)
                        appendVertice(simplex, -axis)
                        if (encloseOrigin()) return true
                        removeVertice(simplex)
                    }
                }
                2 -> {
                    val d = simplex.c[1].w - simplex.c[0].w
                    for (i in 0..2) {
                        val axis = Vec3()
                        axis[i] = 1f
                        val p = d cross axis
                        if (p.length2() > 0) {
                            appendVertice(simplex, p)
                            if (encloseOrigin()) return true
                            removeVertice(simplex)
                            appendVertice(simplex, -p)
                            if (encloseOrigin()) return (true)
                            removeVertice(simplex)
                        }
                    }
                }
                3 -> {
                    val n = (simplex.c[1].w - simplex.c[0].w) cross (simplex.c[2].w - simplex.c[0].w)
                    if (n.length2() > 0) {
                        appendVertice(simplex, n)
                        if (encloseOrigin()) return true
                        removeVertice(simplex)
                        appendVertice(simplex, -n)
                        if (encloseOrigin()) return true
                        removeVertice(simplex)
                    }
                }
                4 -> {
                    if (abs(det(simplex.c[0].w - simplex.c[3].w,
                            simplex.c[1].w - simplex.c[3].w,
                            simplex.c[2].w - simplex.c[3].w)) > 0)
                        return true
                }
            }
            return false
        }

        /* Internals	*/
        private fun getSupport(d: Vec3, sv: SV) {
            sv.d put d / d.length()
            sv.w put shape.support(sv.d)
        }

        private fun removeVertice(simplex: Simplex) = free.set(nfree++, simplex.c[--simplex.rank])
        private fun appendVertice(simplex: Simplex, v: Vec3) {
            simplex.p[simplex.rank] = 0f
            simplex.c[simplex.rank] = free[--nfree]
            getSupport(v, simplex.c[simplex.rank++])
        }

        companion object {

            private fun det(a: Vec3, b: Vec3, c: Vec3) = a.y * b.z * c.x + a.z * b.x * c.y -
                    a.x * b.z * c.y - a.y * b.x * c.z + a.x * b.y * c.z - a.z * b.y * c.x

            private fun projectOrigin(a: Vec3, b: Vec3, w: FloatArray, m: IntArray): Float {
                val d = b - a
                val l = d.length2()
                if (l > GJK_SIMPLEX2_EPS) {
                    val t = if (l > 0) -a.dot(d) / l else 0f
                    return when {
                        t >= 1 -> {
                            w[0] = 0f; w[1] = 1f; m[0] = 2; b.length2()
                        }
                        t <= 0 -> {
                            w[0] = 1f; w[1] = 0f; m[0] = 1; a.length2()
                        }
                        else -> {
                            w[1] = t; w[0] = 1 - w[1]; m[0] = 3; (a + d * t).length2()
                        }
                    }
                }
                return -1f
            }

            private val imd3 = intArrayOf(1, 2, 0)

            private fun projectOrigin(a: Vec3, b: Vec3, c: Vec3, w: FloatArray, m: IntArray): Float {
                val vt = arrayOf(a, b, c)
                val dl = arrayOf(a - b, b - c, c - a)
                val n = dl[0] cross dl[1]
                val l = n.length2()
                if (l > GJK_SIMPLEX3_EPS) {
                    var mindist = -1f
                    val subw = FloatArray(2)
                    var subm = 0
                    for (i in 0..2) {
                        if (vt[i] dot (dl[i] cross n) > 0) {
                            val j = imd3[i]
                            val p = intArrayOf(subm)
                            val subd = projectOrigin(vt[i], vt[j], subw, p)
                            subm = p[0]
                            if ((mindist < 0) || (subd < mindist)) {
                                mindist = subd
                                m[0] = (if (subm has 1) 1 shl i else 0) + if (subm has 2) 1 shl j else 0
                                w[i] = subw[0]
                                w[j] = subw[1]
                                w[imd3[j]] = 0f
                            }
                        }
                    }
                    if (mindist < 0) {
                        val d = a dot n
                        val s = sqrt(l)
                        val p = n * (d / l)
                        mindist = p.length2()
                        m[0] = 7
                        w[0] = (dl[1] cross (b - p)).length() / s
                        w[1] = (dl[2] cross (c - p)).length() / s
                        w[2] = 1 - (w[0] + w[1])
                    }
                    return (mindist)
                }
                return -1f
            }
        }

        val imd3_ = intArrayOf(1, 2, 0)
        fun projectOrigin(a: Vec3, b: Vec3, c: Vec3, d: Vec3, w: FloatArray, m: IntArray): Float {
            val vt = arrayOf(a, b, c, d)
            val dl = arrayOf(a - d, b - d, c - d)
            val vl = det(dl[0], dl[1], dl[2])
            val ng = vl * (a dot (b - c).cross(a - b)) <= 0
            TODO()
//            if (ng && abs(vl) > GJK_SIMPLEX4_EPS)) {
//                btScalar mindist = - 1
//                btScalar subw [3] = { 0.f, 0.f, 0.f }
//                U subm (0)
//                for (U i = 0;i < 3;++i)
//                {
//                    const U j = imd3_[i]
//                    const btScalar s = vl * btDot(d, btCross(dl[i], dl[j]))
//                    if (s > 0) {
//                        const btScalar subd = projectorigin(*vt[i], *vt[j], d, subw, subm)
//                        if ((mindist < 0) || (subd < mindist)) {
//                            mindist = subd
//                            m = static_cast<U>((subm&1?1<<i:0)+
//                            (subm&2?1<<j:0)+
//                            (subm&4?8:0))
//                            w[i] = subw[0]
//                            w[j] = subw[1]
//                            w[imd3_[j]] = 0
//                            w[3] = subw[2]
//                        }
//                    }
//                }
//                if (mindist < 0) {
//                    mindist = 0
//                    m = 15
//                    w[0] = det(c, b, d) / vl
//                    w[1] = det(a, c, d) / vl
//                    w[2] = det(b, a, d) / vl
//                    w[3] = 1 - (w[0] + w[1] + w[2])
//                }
//                return (mindist)
//            }
//            return (-1)
        }
    }

    fun stackSizeRequirement(): Int {
        TODO()
    }

    fun distance(shape0: ConvexShape, trs0: Transform,
                 shape1: ConvexShape, trs1: Transform,
                 guess: Vec3,
                 results: Results): Boolean {

        TODO()
    }

    fun penetration(shape0: ConvexShape, trs0: Transform,
                    shape1: ConvexShape, trs1: Transform,
                    guess: Vec3,
                    results: Results,
                    usemargins: Boolean = true): Boolean {
        TODO()
    }

    fun signedDistance(position: Vec3,
                       margin: Float,
                       shape: ConvexShape,
                       trs: Transform,
                       results: Results): Float {
        TODO()
    }

    fun signedDistance(shape0: ConvexShape, trs0: Transform,
                       shape1: ConvexShape, trs1: Transform,
                       guess: Vec3,
                       results: Results): Boolean {
        TODO()
    }
}