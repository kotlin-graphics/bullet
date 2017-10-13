/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2014 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the
use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
claim that you wrote the original software. If you use this software in a
product, an acknowledgment in the product documentation would be appreciated
but is not required.
2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
Initial GJK-EPA collision solver by Nathanael Presson, 2008
Improvements and refactoring by Erwin Coumans, 2008-2014
*/

package bullet.collision.narrowPhaseCollision

import bullet.ConvexTemplate
import bullet.DistanceTemplate
import bullet.collision.narrowPhaseCollision.GjkEpaSolver3.Results.Status.*
import bullet.has
import bullet.linearMath.Mat3
import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt

object GjkEpaSolver3 {

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

        var status = Invalid
        val witnesses = Array(2, { Vec3() })
        var normal = Vec3()
        var distance = 0f
    }

    // Config

    /* GJK	*/
    val GJK_MAX_ITERATIONS = 128
    val GJK_ACCURARY = 0.0001f
    val GJK_MIN_DISTANCE = 0.0001f
    val GJK_DUPLICATED_EPS = 0.0001f
    val GJK_SIMPLEX2_EPS = 0f
    val GJK_SIMPLEX3_EPS = 0f
    val GJK_SIMPLEX4_EPS = 0f

    /* EPA	*/
    val EPA_MAX_VERTICES = 64
    val EPA_MAX_FACES = EPA_MAX_VERTICES * 2
    val EPA_MAX_ITERATIONS = 255
    val EPA_ACCURACY = 0.0001f
    val EPA_FALLBACK = 10 * EPA_ACCURACY
    val EPA_PLANE_EPS = 0.00001f
    val EPA_INSIDE_EPS = 0.01f

    class MinkowskiDiff(val convexAPtr: ConvexTemplate, val convexBPtr: ConvexTemplate) {

        var toshape1 = Mat3()
        var toshape0 = Transform()

        var enableMargin = false

        fun support0(d: Vec3) = convexAPtr.getLocalSupportWithMargin(d)
        fun support1(d: Vec3) = toshape0 * convexBPtr.getLocalSupportWithMargin(toshape1 * d)

        fun support(d: Vec3) = support0(d) - support1(-d)
        fun support(d: Vec3, index: Int) = if (index != 0) support1(d) else support0(d)
    }

    enum class GjkStatus { Valid, Inside, Failed }

    class GJK(val a: ConvexTemplate, val b: ConvexTemplate) {

        /* Types		*/
        class SV(val d: Vec3 = Vec3(), val w: Vec3 = Vec3())

        class Simplex {
            val c = Array(4, { SV() })
            val p = FloatArray(4)
            var rank = 0
        }

        /* Fields		*/

        var shape = MinkowskiDiff(a, b)
        val ray = Vec3()
        var distance = 0f
        val simplices = Array(2, { Simplex() })
        val store = Array(4, { SV() })
        val free = Array(4, { SV() })
        var nfree = 0
        var current = 0
        lateinit var simplex: Simplex
        var status = GjkStatus.Failed

        fun initialize() {
            ray put 0f
            nfree = 0
            status = GjkStatus.Failed
            current = 0
            distance = 0f
        }

        fun evaluate(shapearg: MinkowskiDiff, guess: Vec3): GjkStatus {
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
            status = GjkStatus.Valid
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
            lastw[0] put ray
            lastw[1] put ray
            lastw[2] put ray
            lastw[3] put ray
            /* Loop						*/
            do {
                val next = 1 - current
                val cs = simplices[current]
                val ns = simplices[next]
                /* Check zero							*/
                val rl = ray.length()
                if (rl < GJK_MIN_DISTANCE) {/* Touching or inside				*/
                    status = GjkStatus.Inside
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
                if (found) {    /* Return old simplex   */
                    removeVertice(simplices[current])
                    break
                } else {  /* Update lastw */
                    clastw = (clastw + 1) and 3
                    lastw[clastw] = w
                }
                /* Check for termination    */
                val omega = (ray dot w) / rl
                alpha = max(omega, alpha)
                if (rl - alpha - GJK_ACCURARY * rl <= 0) {  /* Return old simplex   */
                    removeVertice(simplices[current])
                    break
                }
                /* Reduce simplex						*/
                val weights = FloatArray(4)
                val p = IntArray(1)
                sqdist = when (cs.rank) {
                    2 -> projectOrigin(cs.c[0].w, cs.c[1].w, weights, p)
                    3 -> projectOrigin(cs.c[0].w, cs.c[1].w, cs.c[2].w, weights, p)
                    4 -> projectOrigin(cs.c[0].w, cs.c[1].w, cs.c[2].w, cs.c[3].w, weights, p)
                    else -> throw Error()
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
                    if (mask == 15) status = GjkStatus.Inside
                } else {    /* Return old simplex   */
                    removeVertice(simplices[current])
                    break
                }
                status = if (++iterations < GJK_MAX_ITERATIONS) status else GjkStatus.Failed
            } while (status == GjkStatus.Valid)
            simplex = simplices[current]
            when (status) {
                GjkStatus.Valid -> distance = ray.length()
                GjkStatus.Inside -> distance = 0f
                else -> Unit
            }
            return status
        }

        fun encloseOrigin(): Boolean {
            when (simplex.rank) {
                1 -> for (i in 0..2) {
                    val axis = Vec3()
                    axis[i] = 1f
                    appendVertice(simplex, axis)
                    if (encloseOrigin()) return true
                    removeVertice(simplex)
                    appendVertice(simplex, -axis)
                    if (encloseOrigin()) return true
                    removeVertice(simplex)
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
                            if (encloseOrigin()) return true
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
                4 -> if (abs(det(simplex.c[0].w - simplex.c[3].w, simplex.c[1].w - simplex.c[3].w,
                        simplex.c[2].w - simplex.c[3].w)) > 0)
                    return true
            }
            return false
        }

        /* Internals	*/
        fun getSupport(d: Vec3, sv: SV) {
            sv.d put (d / d.length())
            sv.w put shape.support(sv.d)
        }

        fun removeVertice(simplex: Simplex) = free.set(nfree++, simplex.c[--simplex.rank])

        fun appendVertice(simplex: Simplex, v: Vec3) {
            simplex.p[simplex.rank] = 0f
            simplex.c[simplex.rank] = free[--nfree]
            getSupport(v, simplex.c[simplex.rank++])
        }

        fun det(a: Vec3, b: Vec3, c: Vec3) = a.y * b.z * c.x + a.z * b.x * c.y -
                a.x * b.z * c.y - a.y * b.x * c.z +
                a.x * b.y * c.z - a.z * b.y * c.x

        fun projectOrigin(a: Vec3, b: Vec3, w: FloatArray, m: IntArray): Float {
            val d = b - a
            val l = d.length2()
            if (l > GJK_SIMPLEX2_EPS) {
                val t = if (l > 0) -(a dot d) / l else 0f
                return when {
                    t >= 1 -> {
                        w[0] = 0f; w[1] = 1f; m[0] = 2; b.length2(); }
                    t <= 0 -> {
                        w[0] = 1f; w[1] = 0f; m[0] = 1; a.length2(); }
                    else -> {
                        w[1] = t; w[0] = 1 - w[1]; m[0] = 3; (a + d * t).length2(); }
                }
            }
            return -1f
        }

        private val imd3 = intArrayOf(1, 2, 0)
        fun projectOrigin(a: Vec3, b: Vec3, c: Vec3, w: FloatArray, m: IntArray): Float {

            val vt = arrayOf(a, b, c)
            val dl = arrayOf(a - b, b - c, c - a)
            val n = dl[0] cross dl[1]
            val l = n.length2()
            if (l > GJK_SIMPLEX3_EPS) {
                var mindist = -1f
                val subw = FloatArray(2)
                val pI = IntArray(1)
                for (i in 0..2)
                    if (vt[i] dot (dl[i] cross n) > 0) {
                        val j = imd3[i]
                        val subd = projectOrigin(vt[i], vt[j], subw, pI)
                        val subm = pI[0]
                        if (mindist < 0 || subd < mindist) {
                            mindist = subd
                            m[0] = (if (subm has 1) 1 shl i else 0) + if (subm has 2) 1 shl j else 0
                            w[i] = subw[0]
                            w[j] = subw[1]
                            w[imd3[j]] = 0f
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
                return mindist
            }
            return -1f
        }

        private val imd3_ = intArrayOf(1, 2, 0)
        fun projectOrigin(a: Vec3, b: Vec3, c: Vec3, d: Vec3, w: FloatArray, m: IntArray): Float {

            val vt = arrayOf(a, b, c, d)
            val dl = arrayOf(a - d, b - d, c - d)
            val vl = det(dl[0], dl[1], dl[2])
            val ng = vl * (a dot ((b - c) cross (a - b))) <= 0
            if (ng && abs(vl) > GJK_SIMPLEX4_EPS) {
                var mindist = -1f
                val subw = FloatArray(3)
                val p = IntArray(1)
                for (i in 0..2) {
                    val j = imd3_[i]
                    val s = vl * (d dot (dl[i] cross dl[j]))
                    if (s > 0) {
                        val subd = projectOrigin(vt[i], vt[j], d, subw, p)
                        val subm = p[0]
                        if (mindist < 0 || subd < mindist) {
                            mindist = subd
                            m[0] = (if (subm has 1) 1 shl i else 0) + (if (subm has 2) 1 shl j else 0) + if (subm has 4) 8 else 0
                            w[i] = subw[0]
                            w[j] = subw[1]
                            w[imd3_[j]] = 0f
                            w[3] = subw[2]
                        }
                    }
                }
                if (mindist < 0) {
                    mindist = 0f
                    m[0] = 15
                    w[0] = det(c, b, d) / vl
                    w[1] = det(a, c, d) / vl
                    w[2] = det(b, a, d) / vl
                    w[3] = 1 - (w[0] + w[1] + w[2])
                }
                return mindist
            }
            return -1f
        }
    }


    enum class EpaStatus { Valid, Touching, Degenerated, NonConvex, InvalidHull, OutOfFaces, OutOfVertices,
        AccuraryReached, FallBack, Failed
    }

    class EPA {
        /* Types		*/

        class Face {
            var n = Vec3()
            var d = 0f
            val c = Array(3, { GJK.SV() })
            val f = Array<Face?>(3, { null })
            val l = Array<Face?>(2, { null })
            val e = IntArray(3)
            var pass = 0
        }

        class List(var root: Face? = null, var count: Int = 0)

        class Horizon(var cf: Face? = null, var ff: Face? = null, var nf: Int = 0)

        /* Fields		*/
        var status = EpaStatus.Failed
        var result = GJK.Simplex()
        var normal = Vec3()
        var depth = 0f
        val svStore = Array(EPA_MAX_VERTICES, { GJK.SV() })
        val fcStore = Array(EPA_MAX_FACES, { Face() })
        var nextSv = 0
        val hull = List()
        val stock = List()

        init {
            for (i in 0 until EPA_MAX_FACES) append(stock, fcStore[EPA_MAX_FACES - i - 1])
        }

        /* Methods		*/
        fun bind(fa: Face, ea: Int, fb: Face, eb: Int) {
            fa.e[ea] = eb; fa.f[ea] = fb
            fb.e[eb] = ea; fb.f[eb] = fa
        }

        fun append(list: List, face: Face) {
            face.l[0] = null
            face.l[1] = list.root
            list.root?.l?.set(0, face)
            list.root = face
            ++list.count
        }

        fun remove(list: List, face: Face) {
            face.l[1]?.l?.set(0, face.l[0])
            face.l[0]?.l?.set(1, face.l[1])
            if (face === list.root) list.root = face.l[1]
            --list.count
        }

        fun evaluate(gjk: GJK, guess: Vec3): EpaStatus {
            val simplex = gjk.simplex
            if (simplex.rank > 1 && gjk.encloseOrigin()) {

                /* Clean up				*/
                while (hull.root != null) {
                    val f = hull.root!!
                    remove(hull, f)
                    append(stock, f)
                }
                status = EpaStatus.Valid
                nextSv = 0
                /* Orient simplex		*/
                if (gjk.det(simplex.c[0].w - simplex.c[3].w, simplex.c[1].w - simplex.c[3].w,
                        simplex.c[2].w - simplex.c[3].w) < 0) {
                    val a = simplex.c[0]; simplex.c[0] = simplex.c[1]; simplex.c[1] = a
                    val b = simplex.p[0]; simplex.p[0] = simplex.p[1]; simplex.p[1] = b
                }
                /* Build initial hull	*/
                val tetra = arrayOf(
                        newface(simplex.c[0], simplex.c[1], simplex.c[2], true)!!,
                        newface(simplex.c[1], simplex.c[0], simplex.c[3], true)!!,
                        newface(simplex.c[2], simplex.c[1], simplex.c[3], true)!!,
                        newface(simplex.c[0], simplex.c[2], simplex.c[3], true)!!)
                if (hull.count == 4) {
                    var best = findbest()
                    var outer = best
                    var pass = 0
                    var iterations = 0
                    bind(tetra[0], 0, tetra[1], 0)
                    bind(tetra[0], 1, tetra[2], 0)
                    bind(tetra[0], 2, tetra[3], 0)
                    bind(tetra[1], 1, tetra[3], 2)
                    bind(tetra[1], 2, tetra[2], 1)
                    bind(tetra[2], 2, tetra[3], 1)
                    status = EpaStatus.Valid
                    while (iterations < EPA_MAX_ITERATIONS) {
                        if (nextSv < EPA_MAX_VERTICES) {
                            val horizon = Horizon()
                            val w = svStore[nextSv++]
                            var valid = true
                            best.pass = ++pass
                            gjk.getSupport(best.n, w)
                            val wdist = (best.n dot w.w) - best.d
                            if (wdist > EPA_ACCURACY) {
                                var j = 0
                                while (j < 3 && valid) {
                                    valid = valid && expand(pass, w, best.f[j]!!, best.e[j], horizon)
                                    ++j
                                }
                                if (valid && (horizon.nf >= 3)) {
                                    bind(horizon.cf!!, 1, horizon.ff!!, 2)
                                    remove(hull, best)
                                    append(stock, best)
                                    best = findbest()
                                    outer = best
                                } else {
                                    status = EpaStatus.InvalidHull
                                    break
                                }
                            } else {
                                status = EpaStatus.AccuraryReached
                                break
                            }
                        } else {
                            status = EpaStatus.OutOfVertices
                            break
                        }
                        ++iterations
                    }
                    val projection = outer.n * outer.d
                    normal put outer.n
                    depth = outer.d
                    with(result) {
                        rank = 3
                        c[0] = outer.c[0]
                        c[1] = outer.c[1]
                        c[2] = outer.c[2]
                        p[0] = ((outer.c[1].w - projection) cross (outer.c[2].w - projection)).length()
                        p[1] = ((outer.c[2].w - projection) cross (outer.c[0].w - projection)).length()
                        p[2] = ((outer.c[0].w - projection) cross (outer.c[1].w - projection)).length()
                        val sum = p[0] + p[1] + p[2]
                        p[0] /= sum
                        p[1] /= sum
                        p[2] /= sum
                    }
                    return status
                }
            }
            /* Fallback		*/
            status = EpaStatus.FallBack
            normal = -guess
            val nl = normal.length()
            if (nl > 0)
                normal divAssign nl
            else
                normal put 0f
            depth = 0f
            with(result) {
                rank = 1
                c[0] = simplex.c[0]
                p[0] = 1f
            }
            return status
        }

        /** NOTE: &dist has been removed for the assumption of being always face.d */
        fun getEdgeDist(face: Face, a: GJK.SV, b: GJK.SV): Boolean {
            val ba = b.w - a.w
            val n_ab = ba cross face.n  // Outward facing edge normal direction, on triangle plane
            val a_dot_nab = a.w dot n_ab    // Only care about the sign to determine inside/outside, so not normalization required

            return if (a_dot_nab < 0) {
                // Outside of edge a->b
                val ba_l2 = ba.length2()
                val a_dot_ba = a.w dot ba
                val b_dot_ba = b.w dot ba

                face.d = when {
                    a_dot_ba > 0 -> a.w.length()    // Pick distance vertex a
                    b_dot_ba < 0 -> b.w.length()    // Pick distance vertex b
                    else -> { // Pick distance to edge a->b
                        val a_dot_b = a.w dot b.w
                        sqrt(max((a.w.length2() * b.w.length2() - a_dot_b * a_dot_b) / ba_l2, 0f))
                    }
                }
                true
            } else false
        }

        fun newface(a: GJK.SV, b: GJK.SV, c: GJK.SV, forced: Boolean): Face? {
            stock.root?.let { face ->
                remove(stock, face)
                append(hull, face)
                face.pass = 0
                face.c[0] = a
                face.c[1] = b
                face.c[2] = c
                face.n = (b.w - a.w) cross (c.w - a.w)
                val l = face.n.length()

                if (l > EPA_ACCURACY) {
                    if (!(getEdgeDist(face, a, b) || getEdgeDist(face, b, c) || getEdgeDist(face, c, a)))
                    // Origin projects to the interior of the triangle
                    // Use distance to triangle plane
                        face.d = (a.w dot face.n) / l

                    face.n divAssign l
                    if (forced || face.d >= -EPA_PLANE_EPS)
                        return face
                    else
                        status = EpaStatus.NonConvex
                } else
                    status = EpaStatus.Degenerated

                remove(hull, face)
                append(stock, face)
                return null
            }
            status = if (stock.root != null) EpaStatus.OutOfVertices else EpaStatus.OutOfFaces
            return null
        }

        fun findbest(): Face {
            var minf = hull.root!!
            var mind = minf.d * minf.d
            var f = minf.l[1]
            while (f != null) {
                val sqd = f.d * f.d
                if (sqd < mind) {
                    minf = f
                    mind = sqd
                }
                f = f.l[1]
            }
            return (minf)
        }

        val i1m3 = intArrayOf(1, 2, 0)
        val i2m3 = intArrayOf(2, 0, 1)

        fun expand(pass: Int, w: GJK.SV, f: Face, e: Int, horizon: Horizon): Boolean {
            if (f.pass != pass) {
                val e1 = i1m3[e]
                if (((f.n dot w.w) - f.d) < -EPA_PLANE_EPS) {
                    newface(f.c[e1], f.c[e], w, false)?.let { nf ->
                        bind(nf, 0, f, e)
                        if (horizon.cf != null) bind(horizon.cf!!, 1, nf, 2); else horizon.ff = nf
                        horizon.cf = nf
                        ++horizon.nf
                        return true
                    }
                } else {
                    val e2 = i2m3[e]
                    f.pass = pass
                    if (expand(pass, w, f.f[e1]!!, f.e[e1], horizon) && expand(pass, w, f.f[e2]!!, f.e[e2], horizon)) {
                        remove(hull, f)
                        append(stock, f)
                        return true
                    }
                }
            }
            return false
        }
    }

    fun initialize(a: ConvexTemplate, b: ConvexTemplate, results: Results, shape: MinkowskiDiff) {
        /* Results		*/
        results.witnesses[0] put 0f
        results.witnesses[1] put 0f
        results.status = Separated
        /* Shape		*/
        shape.toshape1 = b.worldTrans.basis transposeTimes a.worldTrans.basis
        shape.toshape0 = a.worldTrans inverseTimes b.worldTrans
    }

    //
    // Api
    //

    fun distance(a: ConvexTemplate, b: ConvexTemplate, guess: Vec3, results: Results): Boolean {
        val shape = MinkowskiDiff(a, b)
        initialize(a, b, results, shape)
        val gjk = GJK(a, b)
        val status = gjk.evaluate(shape, guess)
        if (status == GjkStatus.Valid) {
            val w0 = Vec3()
            val w1 = Vec3()
            for (i in 0 until gjk.simplex.rank) {
                val p = gjk.simplex.p[i]
                w0 += shape.support(gjk.simplex.c[i].d, 0) * p
                w1 += shape.support(-gjk.simplex.c[i].d, 1) * p
            }
            results.witnesses[0] = a.worldTrans * w0
            results.witnesses[1] = a.worldTrans * w1
            results.normal = w0 - w1
            results.distance = results.normal.length()
            results.normal divAssign if (results.distance > GJK_MIN_DISTANCE) results.distance else 1f
            return true
        }
        results.status = if (status == GjkStatus.Inside) Penetrating else GJK_Failed
        return false
    }

    fun penetration(a: ConvexTemplate, b: ConvexTemplate, guess: Vec3, results: Results): Boolean {

        val shape = MinkowskiDiff(a, b)
        initialize(a, b, results, shape)
        val gjk = GJK(a, b)
        val gjkStatus = gjk.evaluate(shape, -guess)
        when (gjkStatus) {
            GjkStatus.Inside -> {
                val epa = EPA()
                val epaStatus = epa.evaluate(gjk, -guess)
                if (epaStatus != EpaStatus.Failed) {
                    val w0 = Vec3()
                    for (i in 0 until epa.result.rank)
                        w0 += shape.support(epa.result.c[i].d, 0) * epa.result.p[i]
                    results.status = Penetrating
                    results.witnesses[0] = a.worldTrans * w0
                    results.witnesses[1] = a.worldTrans * (w0 - epa.normal * epa.depth)
                    results.normal = -epa.normal
                    results.distance = -epa.depth
                    return true
                } else results.status = EPA_Failed
            }
            GjkStatus.Failed -> results.status = EPA_Failed
            else -> Unit
        }
        return false
    }
}

fun computeGjkDistance(a: ConvexTemplate, b: ConvexTemplate, colDesc: GjkCollisionDescription, distInfo: DistanceTemplate): Int {
    val results = GjkEpaSolver3.Results()
    val guess = colDesc.firstDir

    if (GjkEpaSolver3.distance(a, b, guess, results))   // is separated?
        with(distInfo) {
            distance = results.distance
            pointOnA put results.witnesses[0]
            pointOnB put results.witnesses[1]
            normalBtoA put results.normal
            return 0
        }
    return -1
}
