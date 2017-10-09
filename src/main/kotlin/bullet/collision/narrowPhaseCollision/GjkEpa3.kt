package bullet.collision.narrowPhaseCollision

import bullet.collision.narrowPhaseCollision.GjkEpaSolver3.Results.Status.Invalid
import bullet.collision.narrowPhaseCollision.GjkEpaSolver3.Results.Status.Separated
import bullet.linearMath.Mat3
import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import kotlin.math.max

class GjkEpaSolver3 {

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
        val normal = Vec3()
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


    class MinkowskiDiff(val convexAPtr: ConvexInterface, val convexBPtr: ConvexInterface) {

        var toshape1 = Mat3()
        var toshape0 = Transform()

        var enableMargin = false

        fun support0(d: Vec3) = convexAPtr.getLocalSupportWithMargin(d)
        fun support1(d: Vec3) = toshape0 * convexBPtr.getLocalSupportWithMargin(toshape1 * d)

        fun support(d: Vec3) = support0(d) - support1(-d)
        fun support(d: Vec3, index: Int) = if (index != 0) support1(d) else support0(d)
    }

    enum class GjkStatus { Valid, Inside, Failed }

    inner class GJK(val a: ConvexInterface, val b: ConvexInterface) {
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
        var simplex: Simplex
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
            sqdist = sqrl
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
                    val clastw = (clastw + 1) and 3
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
                        case 4:    sqdist = projectorigin(cs.c[0]
                    -> w
                        ,
                    cs.c[1] -> w
                        ,
                    cs.c[2] -> w
                        ,
                    cs.c[3] -> w
                        ,
                    weights, mask)
                }
                val mask = p[0]
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
                    if (mask == 15) m_status = eGjkInside
                } else {/* Return old simplex				*/
                    removevertice(m_simplices[m_current])
                    break
                }
                m_status = ((++iterations) < GJK_MAX_ITERATIONS)?m_status:eGjkFailed
            } while (m_status == eGjkValid)
            m_simplex = & m_simplices [m_current]
            switch(m_status)
            {
                case eGjkValid : m_distance = m_ray . length ();break
                case eGjkInside : m_distance =0;break
                default:
                {
                }
            }
            return (m_status)
        }
        bool EncloseOrigin ()
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
        static btScalar det(const btVector3 & a, const btVector3 & b, const btVector3 & c)
        {
            return (a.y() * b.z() * c.x() + a.z() * b.x() * c.y() -
                    a.x() * b.z() * c.y() - a.y() * b.x() * c.z() +
                    a.x() * b.y() * c.z() - a.z() * b.y() * c.x())
        }

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
        static btScalar projectorigin(const btVector3 & a,
        const btVector3 & b,
        const btVector3 & c,
        btScalar * w, U& m)
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
        static btScalar projectorigin(const btVector3 & a,
        const btVector3 & b,
        const btVector3 & c,
        const btVector3 & d,
        btScalar * w, U& m)
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
//
//
//enum	eEpaStatus
//{
//    eEpaValid,
//    eEpaTouching,
//    eEpaDegenerated,
//    eEpaNonConvex,
//    eEpaInvalidHull,
//    eEpaOutOfFaces,
//    eEpaOutOfVertices,
//    eEpaAccuraryReached,
//    eEpaFallBack,
//    eEpaFailed
//};
//
//
//// EPA
//template <typename btConvexTemplate>
//struct	EPA
//{
//    /* Types		*/
//
//    struct	sFace
//            {
//                btVector3	n;
//                btScalar	d;
//                typename GJK<btConvexTemplate>::sSV*		c[3];
//                sFace*		f[3];
//                sFace*		l[2];
//                U1			e[3];
//                U1			pass;
//            };
//    struct	sList
//            {
//                sFace*		root;
//                U			count;
//                sList() : root(0),count(0)	{}
//            };
//    struct	sHorizon
//            {
//                sFace*		cf;
//                sFace*		ff;
//                U			nf;
//                sHorizon() : cf(0),ff(0),nf(0)	{}
//            };
//
//    /* Fields		*/
//    eEpaStatus		m_status;
//    typename GJK<btConvexTemplate>::sSimplex	m_result;
//    btVector3		m_normal;
//    btScalar		m_depth;
//    typename GJK<btConvexTemplate>::sSV				m_sv_store[EPA_MAX_VERTICES];
//    sFace			m_fc_store[EPA_MAX_FACES];
//    U				m_nextsv;
//    sList			m_hull;
//    sList			m_stock;
//    /* Methods		*/
//    EPA()
//    {
//        Initialize();
//    }
//
//
//    static inline void		bind(sFace* fa,U ea,sFace* fb,U eb)
//    {
//        fa->e[ea]=(U1)eb;fa->f[ea]=fb;
//        fb->e[eb]=(U1)ea;fb->f[eb]=fa;
//    }
//    static inline void		append(sList& list,sFace* face)
//    {
//        face->l[0]	=	0;
//        face->l[1]	=	list.root;
//        if(list.root) list.root->l[0]=face;
//        list.root	=	face;
//        ++list.count;
//    }
//    static inline void		remove(sList& list,sFace* face)
//    {
//        if(face->l[1]) face->l[1]->l[0]=face->l[0];
//        if(face->l[0]) face->l[0]->l[1]=face->l[1];
//        if(face==list.root) list.root=face->l[1];
//        --list.count;
//    }
//
//
//    void				Initialize()
//    {
//        m_status	=	eEpaFailed;
//        m_normal	=	btVector3(0,0,0);
//        m_depth		=	0;
//        m_nextsv	=	0;
//        for(U i=0;i<EPA_MAX_FACES;++i)
//        {
//            append(m_stock,&m_fc_store[EPA_MAX_FACES-i-1]);
//        }
//    }
//    eEpaStatus			Evaluate(GJK<btConvexTemplate>& gjk,const btVector3& guess)
//    {
//        typename GJK<btConvexTemplate>::sSimplex&	simplex=*gjk.m_simplex;
//        if((simplex.rank>1)&&gjk.EncloseOrigin())
//        {
//
//            /* Clean up				*/
//            while(m_hull.root)
//            {
//                sFace*	f = m_hull.root;
//                remove(m_hull,f);
//                append(m_stock,f);
//            }
//            m_status	=	eEpaValid;
//            m_nextsv	=	0;
//            /* Orient simplex		*/
//            if(gjk.det(	simplex.c[0]->w-simplex.c[3]->w,
//            simplex.c[1]->w-simplex.c[3]->w,
//            simplex.c[2]->w-simplex.c[3]->w)<0)
//            {
//                btSwap(simplex.c[0],simplex.c[1]);
//                btSwap(simplex.p[0],simplex.p[1]);
//            }
//            /* Build initial hull	*/
//            sFace*	tetra[]={newface(simplex.c[0],simplex.c[1],simplex.c[2],true),
//                newface(simplex.c[1],simplex.c[0],simplex.c[3],true),
//                newface(simplex.c[2],simplex.c[1],simplex.c[3],true),
//                newface(simplex.c[0],simplex.c[2],simplex.c[3],true)};
//            if(m_hull.count==4)
//            {
//                sFace*		best=findbest();
//                sFace		outer=*best;
//                U			pass=0;
//                U			iterations=0;
//                bind(tetra[0],0,tetra[1],0);
//                bind(tetra[0],1,tetra[2],0);
//                bind(tetra[0],2,tetra[3],0);
//                bind(tetra[1],1,tetra[3],2);
//                bind(tetra[1],2,tetra[2],1);
//                bind(tetra[2],2,tetra[3],1);
//                m_status=eEpaValid;
//                for(;iterations<EPA_MAX_ITERATIONS;++iterations)
//                {
//                    if(m_nextsv<EPA_MAX_VERTICES)
//                    {
//                        sHorizon		horizon;
//                        typename GJK<btConvexTemplate>::sSV*			w=&m_sv_store[m_nextsv++];
//                        bool			valid=true;
//                        best->pass	=	(U1)(++pass);
//                        gjk.getsupport(best->n,*w);
//                        const btScalar	wdist=btDot(best->n,w->w)-best->d;
//                        if(wdist>EPA_ACCURACY)
//                        {
//                            for(U j=0;(j<3)&&valid;++j)
//                            {
//                                valid&=expand(	pass,w,
//                                best->f[j],best->e[j],
//                                horizon);
//                            }
//                            if(valid&&(horizon.nf>=3))
//                            {
//                                bind(horizon.cf,1,horizon.ff,2);
//                                remove(m_hull,best);
//                                append(m_stock,best);
//                                best=findbest();
//                                outer=*best;
//                            } else { m_status=eEpaInvalidHull;break; }
//                        } else { m_status=eEpaAccuraryReached;break; }
//                    } else { m_status=eEpaOutOfVertices;break; }
//                }
//                const btVector3	projection=outer.n*outer.d;
//                m_normal	=	outer.n;
//                m_depth		=	outer.d;
//                m_result.rank	=	3;
//                m_result.c[0]	=	outer.c[0];
//                m_result.c[1]	=	outer.c[1];
//                m_result.c[2]	=	outer.c[2];
//                m_result.p[0]	=	btCross(	outer.c[1]->w-projection,
//                outer.c[2]->w-projection).length();
//                m_result.p[1]	=	btCross(	outer.c[2]->w-projection,
//                outer.c[0]->w-projection).length();
//                m_result.p[2]	=	btCross(	outer.c[0]->w-projection,
//                outer.c[1]->w-projection).length();
//                const btScalar	sum=m_result.p[0]+m_result.p[1]+m_result.p[2];
//                m_result.p[0]	/=	sum;
//                m_result.p[1]	/=	sum;
//                m_result.p[2]	/=	sum;
//                return(m_status);
//            }
//        }
//        /* Fallback		*/
//        m_status	=	eEpaFallBack;
//        m_normal	=	-guess;
//        const btScalar	nl=m_normal.length();
//        if(nl>0)
//            m_normal	=	m_normal/nl;
//        else
//            m_normal	=	btVector3(1,0,0);
//        m_depth	=	0;
//        m_result.rank=1;
//        m_result.c[0]=simplex.c[0];
//        m_result.p[0]=1;
//        return(m_status);
//    }
//    bool getedgedist(sFace* face, typename GJK<btConvexTemplate>::sSV* a, typename GJK<btConvexTemplate>::sSV* b, btScalar& dist)
//    {
//        const btVector3 ba = b->w - a->w;
//        const btVector3 n_ab = btCross(ba, face->n); // Outward facing edge normal direction, on triangle plane
//        const btScalar a_dot_nab = btDot(a->w, n_ab); // Only care about the sign to determine inside/outside, so not normalization required
//
//        if(a_dot_nab < 0)
//        {
//            // Outside of edge a->b
//
//            const btScalar ba_l2 = ba.length2();
//            const btScalar a_dot_ba = btDot(a->w, ba);
//            const btScalar b_dot_ba = btDot(b->w, ba);
//
//            if(a_dot_ba > 0)
//            {
//                // Pick distance vertex a
//                dist = a->w.length();
//            }
//            else if(b_dot_ba < 0)
//            {
//                // Pick distance vertex b
//                dist = b->w.length();
//            }
//            else
//            {
//                // Pick distance to edge a->b
//                const btScalar a_dot_b = btDot(a->w, b->w);
//                dist = btSqrt(btMax((a->w.length2() * b->w.length2() - a_dot_b * a_dot_b) / ba_l2, (btScalar)0));
//            }
//
//            return true;
//        }
//
//        return false;
//    }
//    sFace*				newface(typename GJK<btConvexTemplate>::sSV* a,typename GJK<btConvexTemplate>::sSV* b,typename GJK<btConvexTemplate>::sSV* c,bool forced)
//    {
//        if(m_stock.root)
//        {
//            sFace*	face=m_stock.root;
//            remove(m_stock,face);
//            append(m_hull,face);
//            face->pass	=	0;
//            face->c[0]	=	a;
//            face->c[1]	=	b;
//            face->c[2]	=	c;
//            face->n		=	btCross(b->w-a->w,c->w-a->w);
//            const btScalar	l=face->n.length();
//            const bool		v=l>EPA_ACCURACY;
//
//            if(v)
//            {
//                if(!(getedgedist(face, a, b, face->d) ||
//                getedgedist(face, b, c, face->d) ||
//                getedgedist(face, c, a, face->d)))
//                {
//                    // Origin projects to the interior of the triangle
//                    // Use distance to triangle plane
//                    face->d = btDot(a->w, face->n) / l;
//                }
//
//                face->n /= l;
//                if(forced || (face->d >= -EPA_PLANE_EPS))
//                {
//                    return face;
//                }
//                else
//                m_status=eEpaNonConvex;
//            }
//            else
//                m_status=eEpaDegenerated;
//
//            remove(m_hull, face);
//            append(m_stock, face);
//            return 0;
//
//        }
//        m_status = m_stock.root ? eEpaOutOfVertices : eEpaOutOfFaces;
//        return 0;
//    }
//    sFace*				findbest()
//    {
//        sFace*		minf=m_hull.root;
//        btScalar	mind=minf->d*minf->d;
//        for(sFace* f=minf->l[1];f;f=f->l[1])
//        {
//            const btScalar	sqd=f->d*f->d;
//            if(sqd<mind)
//            {
//                minf=f;
//                mind=sqd;
//            }
//        }
//        return(minf);
//    }
//    bool				expand(U pass,typename GJK<btConvexTemplate>::sSV* w,sFace* f,U e,sHorizon& horizon)
//    {
//        static const U	i1m3[]={1,2,0};
//        static const U	i2m3[]={2,0,1};
//        if(f->pass!=pass)
//        {
//            const U	e1=i1m3[e];
//            if((btDot(f->n,w->w)-f->d)<-EPA_PLANE_EPS)
//            {
//                sFace*	nf=newface(f->c[e1],f->c[e],w,false);
//                if(nf)
//                {
//                    bind(nf,0,f,e);
//                    if(horizon.cf) bind(horizon.cf,1,nf,2); else horizon.ff=nf;
//                    horizon.cf=nf;
//                    ++horizon.nf;
//                    return(true);
//                }
//            }
//            else
//            {
//                const U	e2=i2m3[e];
//                f->pass		=	(U1)pass;
//                if(	expand(pass,w,f->f[e1],f->e[e1],horizon)&&
//                expand(pass,w,f->f[e2],f->e[e2],horizon))
//                {
//                    remove(m_hull,f);
//                    append(m_stock,f);
//                    return(true);
//                }
//            }
//        }
//        return(false);
//    }
//
//};

    fun initialize(a: ConvexInterface, b: ConvexInterface, results: Results, shape: MinkowskiDiff) {
        /* Results		*/
        results.witnesses[0] put 0f
        results.witnesses[1] put 0f
        results.status = Separated
        /* Shape		*/
        shape.toshape1 = b.worldTrans.basis transposeTimes a.worldTrans.basis
        shape.toshape0 = a.worldTrans inverseTimes b.worldTrans
    }
//
//
////
//// Api
////
//
//
//
////
//template <typename btConvexTemplate>
//bool		btGjkEpaSolver3_Distance(const btConvexTemplate& a, const btConvexTemplate& b,
//const btVector3& guess,
//btGjkEpaSolver3::sResults& results)
//{
//    MinkowskiDiff<btConvexTemplate>			shape(a,b);
//    Initialize(a,b,results,shape);
//    GJK<btConvexTemplate>				gjk(a,b);
//    eGjkStatus	gjk_status=gjk.Evaluate(shape,guess);
//    if(gjk_status==eGjkValid)
//    {
//        btVector3	w0=btVector3(0,0,0);
//        btVector3	w1=btVector3(0,0,0);
//        for(U i=0;i<gjk.m_simplex->rank;++i)
//        {
//            const btScalar	p=gjk.m_simplex->p[i];
//            w0+=shape.Support( gjk.m_simplex->c[i]->d,0)*p;
//            w1+=shape.Support(-gjk.m_simplex->c[i]->d,1)*p;
//        }
//        results.witnesses[0]	=	a.getWorldTransform()*w0;
//        results.witnesses[1]	=	a.getWorldTransform()*w1;
//        results.normal			=	w0-w1;
//        results.distance		=	results.normal.length();
//        results.normal			/=	results.distance>GJK_MIN_DISTANCE?results.distance:1;
//        return(true);
//    }
//    else
//    {
//        results.status	=	gjk_status==eGjkInside?
//        btGjkEpaSolver3::sResults::Penetrating	:
//        btGjkEpaSolver3::sResults::GJK_Failed	;
//        return(false);
//    }
//}

    fun penetration(a: ConvexInterface, b: ConvexInterface, guess: Vec3, results: Results) {

        val shape = MinkowskiDiff(a, b)
        initialize(a, b, results, shape)
        GJK<btConvexTemplate> gjk (a, b)
        eGjkStatus gjk_status = gjk . Evaluate (shape, -guess)
        switch(gjk_status)
        {
            case eGjkInside :
            {
                EPA<btConvexTemplate> epa
                        eEpaStatus epa_status = epa . Evaluate (gjk, -guess)
                if (epa_status != eEpaFailed) {
                    btVector3 w0 = btVector3 (0, 0, 0)
                    for (U i = 0;i < epa.m_result.rank;++i)
                    {
                        w0 += shape.Support(epa.m_result.c[i]->d, 0)*epa.m_result.p[i]
                    }
                    results.status = btGjkEpaSolver3::sResults::Penetrating
                    results.witnesses[0] = a.getWorldTransform() * w0
                    results.witnesses[1] = a.getWorldTransform() * (w0 - epa.m_normal * epa.m_depth)
                    results.normal = -epa.m_normal
                    results.distance = -epa.m_depth
                    return (true)
                } else results.status = btGjkEpaSolver3::sResults::EPA_Failed
            }
            break
            case eGjkFailed :
            results.status = btGjkEpaSolver3::sResults::GJK_Failed
            break
            default:
            {
            }
        }
        return (false)
    }
//
//#if 0
//int	btComputeGjkEpaPenetration2(const btCollisionDescription& colDesc, btDistanceInfo* distInfo)
//{
//    btGjkEpaSolver3::sResults results;
//    btVector3 guess = colDesc.m_firstDir;
//
//    bool res = btGjkEpaSolver3::Penetration(colDesc.m_objA,colDesc.m_objB,
//            colDesc.m_transformA,colDesc.m_transformB,
//            colDesc.m_localSupportFuncA,colDesc.m_localSupportFuncB,
//            guess,
//            results);
//    if (res)
//    {
//        if ((results.status==btGjkEpaSolver3::sResults::Penetrating) || results.status==GJK::eStatus::Inside)
//            {
//                //normal could be 'swapped'
//
//                distInfo->m_distance = results.distance;
//                distInfo->m_normalBtoA = results.normal;
//                btVector3 tmpNormalInB = results.witnesses[1]-results.witnesses[0];
//                btScalar lenSqr = tmpNormalInB.length2();
//                if (lenSqr <= (SIMD_EPSILON*SIMD_EPSILON))
//                {
//                    tmpNormalInB = results.normal;
//                    lenSqr = results.normal.length2();
//                }
//
//                if (lenSqr > (SIMD_EPSILON*SIMD_EPSILON))
//                {
//                    tmpNormalInB /= btSqrt(lenSqr);
//                    btScalar distance2 = -(results.witnesses[0]-results.witnesses[1]).length();
//                    //only replace valid penetrations when the result is deeper (check)
//                    //if ((distance2 < results.distance))
//                    {
//                        distInfo->m_distance = distance2;
//                        distInfo->m_pointOnA= results.witnesses[0];
//                        distInfo->m_pointOnB= results.witnesses[1];
//                        distInfo->m_normalBtoA= tmpNormalInB;
//                        return 0;
//                    }
//                }
//            }
//
//    }
//
//    return -1;
//}
//#endif
//
//template <typename btConvexTemplate, typename btDistanceInfoTemplate>
//int	btComputeGjkDistance(const btConvexTemplate& a, const btConvexTemplate& b,
//const btGjkCollisionDescription& colDesc, btDistanceInfoTemplate* distInfo)
//{
//    btGjkEpaSolver3::sResults results;
//    btVector3 guess = colDesc.m_firstDir;
//
//    bool isSeparated = btGjkEpaSolver3_Distance(	a,b,
//    guess,
//    results);
//    if (isSeparated)
//        {
//            distInfo->m_distance = results.distance;
//            distInfo->m_pointOnA= results.witnesses[0];
//            distInfo->m_pointOnB= results.witnesses[1];
//            distInfo->m_normalBtoA= results.normal;
//            return 0;
//        }
//
//    return -1;
//}
}
