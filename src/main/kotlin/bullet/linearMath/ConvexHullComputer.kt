package bullet.linearMath

class ConvexHullComputer {

    class Edge {

        private var next = 0
        private var reverse = 0
        private var targetVertex = 0

        val sourceVertex: Nothing get() = TODO()//(this + reverse)->targetVertex

//        val targetVertex() const
//    {
//        return targetVertex;
//    }
//
//        const Edge* getNextEdgeOfVertex() const // clockwise list of all edges of a vertex
//    {
//        return this + next;
//    }
//
//        const Edge* getNextEdgeOfFace() const // counter-clockwise list of all edges of a face
//    {
//        return (this + reverse)->getNextEdgeOfVertex();
//    }
//
//        const Edge* getReverseEdge() const
//    {
//        return this + reverse;
//    }
    }

    /** Vertices of the output hull */
    val vertices = ArrayList<Vec3>()

    /** Edges of the output hull    */
    val edges = ArrayList<Edge>()

    /** Faces of the convex hull. Each entry is an index into the "edges" array pointing to an edge of the face.
     *  Faces are planar n-gons */
    val faces = ArrayList<Int>()

    /*  Compute convex hull of "count" vertices stored in "coords". "stride" is the difference in bytes between the
        addresses of consecutive vertices. If "shrink" is positive, the convex hull is shrunken by that amount
        (each face is moved by "shrink" length units towards the center along its normal).
        If "shrinkClamp" is positive, "shrink" is clamped to not exceed "shrinkClamp * innerRadius", where "innerRadius"
        is the minimum distance of a face to the center of the convex hull.

        The returned value is the amount by which the hull has been shrunken. If it is negative, the amount was so large
        that the resulting convex hull is empty.

        The output convex hull can be found in the member variables "vertices", "edges", "faces".    */
    fun compute(coords: FloatArray, stride: Int, count: Int, shrink: Float, shrinkClamp: Float): Float {

        if (count <= 0) {
            vertices.clear()
            edges.clear()
            faces.clear()
            return 0f
        }
TODO()
//        val hull = ConvexHullInternal()
//        hull.compute(coords, doubleCoords, stride, count)
//
//        btScalar shift = 0
//        if ((shrink > 0) && ((shift = hull.shrink(shrink, shrinkClamp)) < 0)) {
//            vertices.clear()
//            edges.clear()
//            faces.clear()
//            return shift
//        }
//
//        vertices.resize(0)
//        edges.resize(0)
//        faces.resize(0)
//
//        btAlignedObjectArray < btConvexHullInternal::Vertex * > oldVertices
//                getVertexCopy(hull.vertexList, oldVertices)
//        int copied = 0
//        while (copied < oldVertices.size()) {
//            btConvexHullInternal::Vertex * v = oldVertices[copied]
//            vertices.push_back(hull.getCoordinates(v))
//            btConvexHullInternal::Edge * firstEdge = v->edges
//            if (firstEdge) {
//                int firstCopy = - 1
//                int prevCopy = - 1
//                btConvexHullInternal::Edge * e = firstEdge
//                do {
//                    if (e->copy < 0)
//                    {
//                        int s = edges . size ()
//                        edges.push_back(Edge())
//                        edges.push_back(Edge())
//                        Edge * c = & edges [s]
//                        Edge * r = & edges [s + 1]
//                        e->copy = s
//                        e->reverse->copy = s+1
//                        c->reverse = 1
//                        r->reverse = -1
//                        c->targetVertex = getVertexCopy(e->target, oldVertices)
//                        r->targetVertex = copied
//                        #ifdef DEBUG_CONVEX_HULL
//                            printf("      CREATE: Vertex *%d has edge to *%d\n", copied, c->getTargetVertex())
//                        #endif
//                    }
//                    if (prevCopy >= 0) {
//                        edges[e->copy].next = prevCopy-e->copy
//                    } else {
//                        firstCopy = e->copy
//                    }
//                    prevCopy = e->copy
//                    e = e->next
//                } while (e != firstEdge)
//                edges[firstCopy].next = prevCopy - firstCopy
//            }
//            copied++
//        }
//
//        for (int i = 0; i < copied; i++)
//        {
//            btConvexHullInternal::Vertex * v = oldVertices[i]
//            btConvexHullInternal::Edge * firstEdge = v->edges
//            if (firstEdge) {
//                btConvexHullInternal::Edge * e = firstEdge
//                do {
//                    if (e->copy >= 0)
//                    {
//                        #ifdef DEBUG_CONVEX_HULL
//                            printf("Vertex *%d has edge to *%d\n", i, edges[e->copy].getTargetVertex())
//                        #endif
//                        faces.push_back(e->copy)
//                        btConvexHullInternal::Edge * f = e
//                        do {
//                            #ifdef DEBUG_CONVEX_HULL
//                                    printf("   Face *%d\n", edges[f->copy].getTargetVertex())
//                            #endif
//                            f->copy = -1
//                            f = f->reverse->prev
//                        } while (f != e)
//                    }
//                    e = e->next
//                } while (e != firstEdge)
//            }
//        }
//
//        return shift
    }

//    / same as above, but double precision
//    btScalar compute(const double* coords, int stride, int count, btScalar shrink, btScalar shrinkClamp)
//    {
//        return compute(coords, true, stride, count, shrink, shrinkClamp);
//    }
}