package bullet.collision.narrowPhaseCollision

import bullet.collision.collisionShapes.ConvexShape
import bullet.linearMath.Mat3
import bullet.linearMath.Transform
import bullet.linearMath.Vec3

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
        val simplex = Simplex()
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
            var sqdist = 0f
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
            sqdist = sqrl
            lastw[0] = Vec3(ray)
            lastw[1] = Vec3(ray)
            lastw[2] = Vec3(ray)
            lastw[3] = Vec3(ray)
            /* Loop						*/
            do {
                const U next = 1 - m_current
                sSimplex&    cs = m_simplices[m_current]
                sSimplex&    ns = m_simplices[next]
                /* Check zero							*/
                const btScalar rl = m_ray.length()
                if (rl < GJK_MIN_DISTANCE) {/* Touching or inside				*/
                    m_status = eStatus::Inside
                    break
                }
                /* Append new vertice in -'v' direction	*/
                appendvertice(cs, -m_ray)
                const btVector3 & w = cs . c [cs.rank - 1]->w
                bool found =false
                for (U i = 0;i < 4;++i)
                {
                    if ((w - lastw[i]).length2() < GJK_DUPLICATED_EPS) {
                        found = true;break; }
                }
                if (found) {/* Return old simplex				*/
                    removevertice(m_simplices[m_current])
                    break
                } else {/* Update lastw					*/
                    lastw[clastw = (clastw + 1)&3] = w
                }
                /* Check for termination				*/
                const btScalar omega = btDot(m_ray, w) / rl
                alpha = btMax(omega, alpha)
                if (((rl - alpha) - (GJK_ACCURACY * rl)) <= 0) {/* Return old simplex				*/
                    removevertice(m_simplices[m_current])
                    break
                }
                /* Reduce simplex						*/
                btScalar weights [4]
                U mask =0
                switch(cs.rank)
                {
                    case    2:    sqdist = projectorigin(cs.c[0]->w,
                    cs.c[1]->w,
                    weights, mask);break
                    case    3:    sqdist = projectorigin(cs.c[0]->w,
                    cs.c[1]->w,
                    cs.c[2]->w,
                    weights, mask);break
                    case    4:    sqdist = projectorigin(cs.c[0]->w,
                    cs.c[1]->w,
                    cs.c[2]->w,
                    cs.c[3]->w,
                    weights, mask);break
                }
                if (sqdist >= 0) {/* Valid	*/
                    ns.rank = 0
                    m_ray = btVector3(0, 0, 0)
                    m_current = next
                    for (U i = 0, ni = cs.rank;i < ni;++i)
                    {
                        if (mask&(1<<i))
                        {
                            ns.c[ns.rank] = cs.c[i]
                            ns.p[ns.rank++] = weights[i]
                            m_ray += cs.c[i]->w*weights[i]
                        }
                        else
                        {
                            m_free[m_nfree++] = cs.c[i]
                        }
                    }
                    if (mask == 15) m_status = eStatus::Inside
                } else {/* Return old simplex				*/
                    removevertice(m_simplices[m_current])
                    break
                }
                m_status = ((++iterations) < GJK_MAX_ITERATIONS)?m_status:eStatus::Failed
            } while (m_status == eStatus::Valid)
            m_simplex = & m_simplices [m_current]
            switch(m_status)
            {
                case eStatus ::Valid:        m_distance = m_ray.length();break
                case eStatus ::Inside:    m_distance = 0;break
                default:
                {
                }
            }
            return (m_status)
        }
        bool                    EncloseOrigin()
        {
            switch(m_simplex->rank)
            {
                case    1:
                {
                    for (U i = 0;i < 3;++i)
                    {
                        btVector3 axis = btVector3 (0, 0, 0)
                        axis[i] = 1
                        appendvertice(*m_simplex, axis)
                        if (EncloseOrigin()) return (true)
                        removevertice(*m_simplex)
                        appendvertice(*m_simplex, -axis)
                        if (EncloseOrigin()) return (true)
                        removevertice(*m_simplex)
                    }
                }
                break
                case    2:
                {
                    const btVector3 d = m_simplex->c[1]->w-m_simplex->c[0]->w
                    for (U i = 0;i < 3;++i)
                    {
                        btVector3 axis = btVector3 (0, 0, 0)
                        axis[i] = 1
                        const btVector3 p = btCross(d, axis)
                        if (p.length2() > 0) {
                            appendvertice(*m_simplex, p)
                            if (EncloseOrigin()) return (true)
                            removevertice(*m_simplex)
                            appendvertice(*m_simplex, -p)
                            if (EncloseOrigin()) return (true)
                            removevertice(*m_simplex)
                        }
                    }
                }
                break
                case    3:
                {
                    const btVector3 n = btCross(m_simplex->c[1]->w-m_simplex->c[0]->w,
                    m_simplex->c[2]->w-m_simplex->c[0]->w)
                    if (n.length2() > 0) {
                        appendvertice(*m_simplex, n)
                        if (EncloseOrigin()) return (true)
                        removevertice(*m_simplex)
                        appendvertice(*m_simplex, -n)
                        if (EncloseOrigin()) return (true)
                        removevertice(*m_simplex)
                    }
                }
                break
                case    4:
                {
                    if (btFabs(det(m_simplex->c[0]->w-m_simplex->c[3]->w,
                    m_simplex->c[1]->w-m_simplex->c[3]->w,
                    m_simplex->c[2]->w-m_simplex->c[3]->w))>0)
                    return (true)
                }
                break
            }
            return (false)
        }
        /* Internals	*/
        void                getsupport(const btVector3& d,sSV& sv)
        const
        {
            sv.d = d / d.length()
            sv.w = m_shape.Support(sv.d)
        }
        void                removevertice(sSimplex& simplex)
        {
            m_free[m_nfree++] = simplex.c[--simplex.rank]
        }
        void                appendvertice(sSimplex& simplex,const btVector3& v)
        {
            simplex.p[simplex.rank] = 0
            simplex.c[simplex.rank] = m_free[--m_nfree]
            getsupport(v, *simplex.c[simplex.rank++])
        }
        static btScalar        det(const btVector3& a,const btVector3& b,const btVector3& c)
        {
            return (a.y() * b.z() * c.x() + a.z() * b.x() * c.y() -
                    a.x() * b.z() * c.y() - a.y() * b.x() * c.z() +
                    a.x() * b.y() * c.z() - a.z() * b.y() * c.x())
        }
        static btScalar        projectorigin(    const btVector3& a,
        const btVector3& b,
        btScalar* w,U& m)
        {
            const btVector3 d = b - a
            const btScalar l = d.length2()
            if (l > GJK_SIMPLEX2_EPS) {
                const btScalar t(l > 0? - btDot(a, d) / l:0)
                if (t >= 1) {
                    w[0] = 0;w[1] = 1;m = 2;return (b.length2()); } else if (t <= 0) {
                    w[0] = 1;w[1] = 0;m = 1;return (a.length2()); } else {
                    w[0] = 1 - (w[1] = t);m = 3;return ((a + d * t).length2()); }
            }
            return (-1)
        }
        static btScalar        projectorigin(    const btVector3& a,
        const btVector3& b,
        const btVector3& c,
        btScalar* w,U& m)
        {
            static const U imd3 [] = { 1, 2, 0 }
            const btVector3 * vt [] = { &a, &b, &c }
            const btVector3 dl[] = { a - b, b-c, c-a }
            const btVector3 n = btCross(dl[0], dl[1])
            const btScalar l = n.length2()
            if (l > GJK_SIMPLEX3_EPS) {
                btScalar mindist = - 1
                btScalar subw [2] = { 0.f, 0.f }
                U subm (0)
                for (U i = 0;i < 3;++i)
                {
                    if (btDot(*vt[i], btCross(dl[i], n)) > 0) {
                        const U j = imd3[i]
                        const btScalar subd(projectorigin(*vt[i], *vt[j], subw, subm))
                        if ((mindist < 0) || (subd < mindist)) {
                            mindist = subd
                            m = static_cast<U>(((subm&1)?1<<i:0)+((subm&2)?1<<j:0))
                            w[i] = subw[0]
                            w[j] = subw[1]
                            w[imd3[j]] = 0
                        }
                    }
                }
                if (mindist < 0) {
                    const btScalar d = btDot(a, n)
                    const btScalar s = btSqrt(l)
                    const btVector3 p = n * (d / l)
                    mindist = p.length2()
                    m = 7
                    w[0] = (btCross(dl[1], b - p)).length() / s
                    w[1] = (btCross(dl[2], c - p)).length() / s
                    w[2] = 1 - (w[0] + w[1])
                }
                return (mindist)
            }
            return (-1)
        }
        static btScalar        projectorigin(    const btVector3& a,
        const btVector3& b,
        const btVector3& c,
        const btVector3& d,
        btScalar* w,U& m)
        {
            static const U imd3 [] = { 1, 2, 0 }
            const btVector3 * vt [] = { &a, &b, &c, &d }
            const btVector3 dl[] = { a - d, b-d, c-d }
            const btScalar vl = det(dl[0], dl[1], dl[2])
            const bool ng = (vl * btDot(a, btCross(b - c, a - b))) <= 0
            if (ng && (btFabs(vl) > GJK_SIMPLEX4_EPS)) {
                btScalar mindist = - 1
                btScalar subw [3] = { 0.f, 0.f, 0.f }
                U subm (0)
                for (U i = 0;i < 3;++i)
                {
                    const U j = imd3[i]
                    const btScalar s = vl * btDot(d, btCross(dl[i], dl[j]))
                    if (s > 0) {
                        const btScalar subd = projectorigin(*vt[i], *vt[j], d, subw, subm)
                        if ((mindist < 0) || (subd < mindist)) {
                            mindist = subd
                            m = static_cast<U>((subm&1?1<<i:0)+
                            (subm&2?1<<j:0)+
                            (subm&4?8:0))
                            w[i] = subw[0]
                            w[j] = subw[1]
                            w[imd3[j]] = 0
                            w[3] = subw[2]
                        }
                    }
                }
                if (mindist < 0) {
                    mindist = 0
                    m = 15
                    w[0] = det(c, b, d) / vl
                    w[1] = det(a, c, d) / vl
                    w[2] = det(b, a, d) / vl
                    w[3] = 1 - (w[0] + w[1] + w[2])
                }
                return (mindist)
            }
            return (-1)
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