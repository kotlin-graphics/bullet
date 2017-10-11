package bullet.linearMath

class ConvexHullInternal {

//    btVector3 scaling;
//    btVector3 center;
//    Pool<Vertex> vertexPool;
//    Pool<Edge> edgePool;
//    Pool<Face> facePool;
//    btAlignedObjectArray<Vertex*> originalVertices;
//    int mergeStamp;
//    int minAxis;
//    int medAxis;
//    int maxAxis;
//    int usedEdgePairs;
//    int maxUsedEdgePairs;
//
//    static Orientation getOrientation(const Edge* prev, const Edge* next, const Point32& s, const Point32& t);
//    Edge* findMaxAngle(bool ccw, const Vertex* start, const Point32& s, const Point64& rxs, const Point64& sxrxs, Rational64& minCot);
//    void findEdgeForCoplanarFaces(Vertex* c0, Vertex* c1, Edge*& e0, Edge*& e1, Vertex* stop0, Vertex* stop1);
//
//    Edge* newEdgePair(Vertex* from, Vertex* to);
//
//    void removeEdgePair(Edge* edge)
//    {
//        Edge* n = edge->next;
//        Edge* r = edge->reverse;
//
//        btAssert(edge->target && r->target);
//
//        if (n != edge)
//            {
//                n->prev = edge->prev;
//                edge->prev->next = n;
//                r->target->edges = n;
//            }
//        else
//            {
//                r->target->edges = NULL;
//            }
//
//        n = r->next;
//
//        if (n != r)
//            {
//                n->prev = r->prev;
//                r->prev->next = n;
//                edge->target->edges = n;
//            }
//        else
//            {
//                edge->target->edges = NULL;
//            }
//
//        edgePool.freeObject(edge);
//        edgePool.freeObject(r);
//        usedEdgePairs--;
//    }
//
//    void computeInternal(int start, int end, IntermediateHull& result);
//
//    bool mergeProjection(IntermediateHull& h0, IntermediateHull& h1, Vertex*& c0, Vertex*& c1);
//
//    void merge(IntermediateHull& h0, IntermediateHull& h1);
//
//    btVector3 toBtVector(const Point32& v);
//
//    btVector3 getBtNormal(Face* face);
//
//    bool shiftFace(Face* face, btScalar amount, btAlignedObjectArray<Vertex*> stack);
//
//    public:
//    Vertex* vertexList;

    fun compute(coords: List<Vec3>, stride: String, count: Int) {

        val min = Vec3(1e30f)
        val max = Vec3(-1e30f)
//        const char * ptr =(const char *) coords;
TODO()
//        for (i in 0 until count) {
//            val p = Vec3 (v[0], v[1], v[2]);
//            ptr += stride;
//            min.setMin(p);
//            max.setMax(p);
//        }
//
//        btVector3 s = max -min;
//        maxAxis = s.maxAxis();
//        minAxis = s.minAxis();
//        if (minAxis == maxAxis) {
//            minAxis = (maxAxis + 1) % 3;
//        }
//        medAxis = 3 - maxAxis - minAxis;
//
//        s /= btScalar(10216);
//        if (((medAxis + 1) % 3) != maxAxis) {
//            s *= -1;
//        }
//        scaling = s;
//
//        if (s[0] != 0) {
//            s[0] = btScalar(1) / s[0];
//        }
//        if (s[1] != 0) {
//            s[1] = btScalar(1) / s[1];
//        }
//        if (s[2] != 0) {
//            s[2] = btScalar(1) / s[2];
//        }
//
//        center = (min + max) * btScalar(0.5);
//
//        btAlignedObjectArray<Point32> points;
//        points.resize(count);
//        ptr = (const char *) coords;
//        if (doubleCoords) {
//            for (int i = 0; i < count; i++)
//            {
//                const double * v =(const double *) ptr;
//                btVector3 p ((btScalar) v [0], (btScalar) v[1], (btScalar) v[2]);
//                ptr += stride;
//                p = (p - center) * s;
//                points[i].x = (int32_t) p [medAxis];
//                points[i].y = (int32_t) p [maxAxis];
//                points[i].z = (int32_t) p [minAxis];
//                points[i].index = i;
//            }
//        } else {
//            for (int i = 0; i < count; i++)
//            {
//                const float * v =(const float *) ptr;
//                btVector3 p (v[0], v[1], v[2]);
//                ptr += stride;
//                p = (p - center) * s;
//                points[i].x = (int32_t) p [medAxis];
//                points[i].y = (int32_t) p [maxAxis];
//                points[i].z = (int32_t) p [minAxis];
//                points[i].index = i;
//            }
//        }
//        points.quickSort(pointCmp());
//
//        vertexPool.reset();
//        vertexPool.setArraySize(count);
//        originalVertices.resize(count);
//        for (int i = 0; i < count; i++)
//        {
//            Vertex * v = vertexPool.newObject();
//            v->edges = NULL;
//            v->point = points[i];
//            v->copy = -1;
//            originalVertices[i] = v;
//        }
//
//        points.clear();
//
//        edgePool.reset();
//        edgePool.setArraySize(6 * count);
//
//        usedEdgePairs = 0;
//        maxUsedEdgePairs = 0;
//
//        mergeStamp = -3;
//
//        IntermediateHull hull;
//        computeInternal(0, count, hull);
//        vertexList = hull.minXy;
    }

//    btVector3 getCoordinates(const Vertex* v);
//
//    btScalar shrink(btScalar amount, btScalar clampAmount);
}