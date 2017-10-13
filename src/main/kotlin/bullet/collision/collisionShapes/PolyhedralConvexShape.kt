package bullet.collision.collisionShapes

import bullet.linearMath.*
import kotlin.math.min
import kotlin.math.sqrt

abstract class PolyhedralConvexShape : ConvexInternalShape() {

    var polyhedron: ConvexPolyhedron? = null

    /** optional method mainly used to generate multiple contact points by clipping polyhedral features (faces/edges)
     *  experimental/work-in-progress   */
    open fun initializePolyhedralFeatures(shiftVerticesByMargin: Int = 0): Boolean {

        polyhedron = ConvexPolyhedron()

        val orgVertices = Array(numVertices, { Vec3().apply { getVertex(it, this) } })

        TODO()
//        btConvexHullComputer conv;
//
//        if (shiftVerticesByMargin) {
//            btAlignedObjectArray<Vec3> planeEquations;
//            btGeometryUtil::getPlaneEquationsFromVertices(orgVertices, planeEquations);
//
//            btAlignedObjectArray<Vec3> shiftedPlaneEquations;
//            for (int p = 0;p < planeEquations.size();p++)
//            {
//                Vec3 plane = planeEquations [p];
//                //	   btScalar margin = getMargin();
//                plane[3] -= getMargin();
//                shiftedPlaneEquations.push_back(plane);
//            }
//
//            btAlignedObjectArray<Vec3> tmpVertices;
//
//            btGeometryUtil::getVerticesFromPlaneEquations(shiftedPlaneEquations, tmpVertices);
//
//            conv.compute(& tmpVertices [0].getX(), sizeof(Vec3), tmpVertices.size(), 0.f, 0.f);
//        } else {
//
//            conv.compute(& orgVertices [0].getX(), sizeof(Vec3), orgVertices.size(), 0.f, 0.f);
//        }
//
//
//
//        btAlignedObjectArray<Vec3> faceNormals;
//        int numFaces = conv . faces . size ();
//        faceNormals.resize(numFaces);
//        btConvexHullComputer * convexUtil = & conv;
//
//
//        btAlignedObjectArray<btFace> tmpFaces;
//        tmpFaces.resize(numFaces);
//
//        int numVertices = convexUtil->vertices.size();
//        m_polyhedron->m_vertices.resize(numVertices);
//        for (int p = 0;p < numVertices;p++)
//        { m_polyhedron ->
//            m_vertices[p] = convexUtil->vertices[p];
//        }
//
//
//        for (int i = 0;i < numFaces;i++)
//        {
//            int face = convexUtil->faces[i];
//            //printf("face=%d\n",face);
//            const btConvexHullComputer ::Edge * firstEdge = & convexUtil->edges[face];
//            const btConvexHullComputer ::Edge * edge = firstEdge;
//
//            Vec3 edges [3];
//            int numEdges = 0;
//            //compute face normals
//
//            do {
//
//                int src = edge->getSourceVertex();
//                tmpFaces[i].m_indices.push_back(src);
//                int targ = edge->getTargetVertex();
//                Vec3 wa = convexUtil->vertices[src];
//
//                Vec3 wb = convexUtil->vertices[targ];
//                Vec3 newEdge = wb -wa;
//                newEdge.normalize();
//                if (numEdges < 2)
//                    edges[numEdges++] = newEdge;
//
//                edge = edge->getNextEdgeOfFace();
//            } while (edge != firstEdge);
//
//            btScalar planeEq = 1e30f;
//
//
//            if (numEdges == 2) {
//                faceNormals[i] = edges[0].cross(edges[1]);
//                faceNormals[i].normalize();
//                tmpFaces[i].m_plane[0] = faceNormals[i].getX();
//                tmpFaces[i].m_plane[1] = faceNormals[i].getY();
//                tmpFaces[i].m_plane[2] = faceNormals[i].getZ();
//                tmpFaces[i].m_plane[3] = planeEq;
//
//            } else {
//                btAssert(0);//degenerate?
//                faceNormals[i].setZero();
//            }
//
//            for (int v = 0;v < tmpFaces[i].m_indices.size();v++)
//            {
//                btScalar eq = m_polyhedron->m_vertices[tmpFaces[i].m_indices[v]].dot(faceNormals[i]);
//                if (planeEq > eq) {
//                    planeEq = eq;
//                }
//            }
//            tmpFaces[i].m_plane[3] = -planeEq;
//        }
//
//        //merge coplanar faces and copy them to m_polyhedron
//
//        btScalar faceWeldThreshold = 0.999f;
//        btAlignedObjectArray<int> todoFaces;
//        for (int i = 0;i < tmpFaces.size();i++)
//        todoFaces.push_back(i);
//
//        while (todoFaces.size()) {
//            btAlignedObjectArray<int> coplanarFaceGroup;
//            int refFace = todoFaces [todoFaces.size() - 1];
//
//            coplanarFaceGroup.push_back(refFace);
//            btFace& faceA = tmpFaces[refFace];
//            todoFaces.pop_back();
//
//            Vec3 faceNormalA (faceA.m_plane[0], faceA.m_plane[1], faceA.m_plane[2]);
//            for (int j = todoFaces.size() - 1;j >= 0;j--)
//            {
//                int i = todoFaces [j];
//                btFace& faceB = tmpFaces[i];
//                Vec3 faceNormalB (faceB.m_plane[0], faceB.m_plane[1], faceB.m_plane[2]);
//                if (faceNormalA.dot(faceNormalB) > faceWeldThreshold) {
//                    coplanarFaceGroup.push_back(i);
//                    todoFaces.remove(i);
//                }
//            }
//
//
//            bool did_merge = false;
//            if (coplanarFaceGroup.size() > 1) {
//                //do the merge: use Graham Scan 2d convex hull
//
//                btAlignedObjectArray<GrahamVector3> orgpoints;
//                Vec3 averageFaceNormal (0, 0, 0);
//
//                for (int i = 0;i < coplanarFaceGroup.size();i++)
//                {
//                    //				m_polyhedron->m_faces.push_back(tmpFaces[coplanarFaceGroup[i]]);
//
//                    btFace& face = tmpFaces[coplanarFaceGroup[i]];
//                    Vec3 faceNormal (face.m_plane[0], face.m_plane[1], face.m_plane[2]);
//                    averageFaceNormal += faceNormal;
//                    for (int f = 0;f < face.m_indices.size();f++)
//                    {
//                        int orgIndex = face . m_indices [f];
//                        Vec3 pt = m_polyhedron->m_vertices[orgIndex];
//
//                        bool found = false;
//
//                        for (int i = 0;i < orgpoints.size();i++)
//                        {
//                            //if ((orgpoints[i].m_orgIndex == orgIndex) || ((rotatedPt-orgpoints[i]).length2()<0.0001))
//                            if (orgpoints[i].m_orgIndex == orgIndex) {
//                                found = true;
//                                break;
//                            }
//                        }
//                        if (!found)
//                            orgpoints.push_back(GrahamVector3(pt, orgIndex));
//                    }
//                }
//
//
//
//                btFace combinedFace;
//                for (int i = 0;i < 4;i++)
//                combinedFace.m_plane[i] = tmpFaces[coplanarFaceGroup[0]].m_plane[i];
//
//                btAlignedObjectArray<GrahamVector3> hull;
//
//                averageFaceNormal.normalize();
//                GrahamScanConvexHull2D(orgpoints, hull, averageFaceNormal);
//
//                for (int i = 0;i < hull.size();i++)
//                {
//                    combinedFace.m_indices.push_back(hull[i].m_orgIndex);
//                    for (int k = 0; k < orgpoints.size(); k++)
//                    {
//                        if (orgpoints[k].m_orgIndex == hull[i].m_orgIndex) {
//                            orgpoints[k].m_orgIndex = -1; // invalidate...
//                            break;
//                        }
//                    }
//                }
//
//                // are there rejected vertices?
//                bool reject_merge = false;
//
//
//
//                for (int i = 0; i < orgpoints.size(); i++) {
//                    if (orgpoints[i].m_orgIndex == -1)
//                        continue; // this is in the hull...
//                    // this vertex is rejected -- is anybody else using this vertex?
//                    for (int j = 0; j < tmpFaces.size(); j++) {
//
//                    btFace& face = tmpFaces[j];
//                    // is this a face of the current coplanar group?
//                    bool is_in_current_group = false;
//                    for (int k = 0; k < coplanarFaceGroup.size(); k++) {
//                    if (coplanarFaceGroup[k] == j) {
//                        is_in_current_group = true;
//                        break;
//                    }
//                }
//                    if (is_in_current_group) // ignore this face...
//                        continue;
//                    // does this face use this rejected vertex?
//                    for (int v = 0; v < face.m_indices.size(); v++) {
//                    if (face.m_indices[v] == orgpoints[i].m_orgIndex) {
//                        // this rejected vertex is used in another face -- reject merge
//                        reject_merge = true;
//                        break;
//                    }
//                }
//                    if (reject_merge)
//                        break;
//                }
//                    if (reject_merge)
//                        break;
//                }
//
//                if (!reject_merge) {
//                    // do this merge!
//                    did_merge = true;
//                    m_polyhedron->m_faces.push_back(combinedFace);
//                }
//            }
//            if (!did_merge) {
//                for (int i = 0;i < coplanarFaceGroup.size();i++)
//                {
//                    btFace face = tmpFaces [coplanarFaceGroup[i]];
//                    m_polyhedron->m_faces.push_back(face);
//                }
//
//            }
//
//
//        }
//
//        m_polyhedron->initialize();
//
//        return true;
    }

    //brute force implementations

    override fun localGetSupportingVertexWithoutMargin(vec: Vec3): Vec3 {

        val supVec = Vec3()
        var maxDot = -LARGE_FLOAT

        val vec = Vec3(vec)
        val lenSqr = vec.length2()
        if (lenSqr < 0.0001f)
            vec.put(1f, 0f, 0f)
        else
            vec *= 1f / sqrt(lenSqr)    // rlen

        val vtx = Vec3()
        val p = FloatArray(1)
        for (k in 0 until numVertices step 128) {
            val temp = Array(128, { Vec3() })
            val innerCount = min(numVertices - k, 128)
            var i = 0
            while (i < innerCount) {
                getVertex(i, temp[i])
                i++
            }
            i = vec.maxDot(temp, innerCount, p)
            val newDot = p[0]
            if (newDot > maxDot) {
                maxDot = newDot
                supVec put temp[i]
            }
        }
        return supVec
    }

    override fun batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: Array<Vec3>, supportVerticesOut: Array<Vec3>,
                                                                   numVectors: Int) {

        var i = 0

        val vtx = Vec3()
        val p = FloatArray(1)

        while (i < numVectors) {
            supportVerticesOut[i][3] = -LARGE_FLOAT
            i++
        }

        for (j in 0 until numVectors) {
            val vec = vectors[j]

            for (k in 0 until numVertices step 128) {
                val temp = Array(128, { Vec3() })
                val innerCount = min(numVertices - k, 128)
                i = 0
                while (i < innerCount) {
                    getVertex(i, temp[i])
                    i++
                }
                i = vec.maxDot(temp, innerCount, p)
                val newDot = p[0]
                if (newDot > supportVerticesOut[j][3]) {
                    supportVerticesOut[j] = temp[i]
                    supportVerticesOut[j][3] = newDot
                }
            }
        }
    }

    override fun calculateLocalInertia(mass: Float, inertia: Vec3) {
        //not yet, return box inertia

        val ident = Transform()
        ident.setIdentity()
        val aabbMin = Vec3()
        val aabbMax = Vec3()
        getAabb(ident, aabbMin, aabbMax)
        val halfExtents = (aabbMax - aabbMin) * 0.5f

        val lx = 2f * (halfExtents.x + margin)
        val ly = 2f * (halfExtents.y + margin)
        val lz = 2f * (halfExtents.z + margin)
        val x2 = lx * lx
        val y2 = ly * ly
        val z2 = lz * lz
        val scaledmass = mass * 0.08333333f

        inertia put (scaledmass * Vec3(y2 + z2, x2 + z2, x2 + y2))
    }


    open val numVertices get() = 0
    open val numEdges get() = 0
    open fun getEdge(i: Int, pa: Vec3, pb: Vec3) {}
    open fun getVertex(i: Int, vtx: Vec3) {}
    open val numPlanes get() = 0
    open fun getPlane(planeNormal: Vec3, planeSupport: Vec3, i: Int) {}
//	virtual int getIndex(int i) const = 0 ;

    open fun isInside(pt: Vec3, tolerance: Float) = false
}

/** The btPolyhedralConvexAabbCachingShape adds aabb caching to the btPolyhedralConvexShape */
open class PolyhedralConvexAabbCachingShape : PolyhedralConvexShape() {

    val localAabbMin = Vec3(1f)
    val localAabbMax = Vec3(-1f)
    var isLocalAabbValid = false

    fun setCachedLocalAabb(aabbMin: Vec3, aabbMax: Vec3) {
        isLocalAabbValid = true
        localAabbMin put aabbMin
        localAabbMax put aabbMax
    }

    fun getCachedLocalAabb(aabbMin: Vec3, aabbMax: Vec3) {
        assert(isLocalAabbValid)
        aabbMin put localAabbMin
        aabbMax put localAabbMax
    }

    fun getNonvirtualAabb(trans: Transform, aabbMin: Vec3, aabbMax: Vec3, margin: Float) {
        //lazy evaluation of local aabb
        assert(isLocalAabbValid)
        transformAabb(localAabbMin, localAabbMax, margin, trans, aabbMin, aabbMax)
    }

    override var localScaling
        get() = super.localScaling
        set(value) {
            super.localScaling = value
            recalcLocalAabb()
        }

    override fun getAabb(trans: Transform, aabbMin: Vec3, aabbMax: Vec3) = getNonvirtualAabb(trans, aabbMin, aabbMax, margin)

    fun recalcLocalAabb() {

        isLocalAabbValid = true

        val _supporting = arrayOf(
                Vec3(0f, 0f, 0f),
                Vec3(0f, 0f, 0f),
                Vec3(0f, 0f, 0f),
                Vec3(0f, 0f, 0f),
                Vec3(0f, 0f, 0f),
                Vec3(0f, 0f, 0f))

        batchedUnitVectorGetSupportingVertexWithoutMargin(_directions, _supporting, 6)

        for (i in 0..2) {
            localAabbMax[i] = _supporting[i][i] + collisionMargin
            localAabbMin[i] = _supporting[i + 3][i] - collisionMargin
        }
    }

    companion object {
        private val _directions = arrayOf(
                Vec3(1f, 0f, 0f),
                Vec3(0f, 1f, 0f),
                Vec3(0f, 0f, 1f),
                Vec3(-1f, 0f, 0f),
                Vec3(0f, -1f, 0f),
                Vec3(0f, 0f, -1f))
    }
}