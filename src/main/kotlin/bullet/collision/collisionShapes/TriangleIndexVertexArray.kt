/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

package bullet.collision.collisionShapes

import bullet.linearMath.Vec3

/** The IndexedMesh indexes a single vertex and index array. Multiple IndexedMesh objects can be passed into a
 *  TriangleIndexVertexArray using addIndexedMesh.
 *  Instead of the number of indices, we pass the number of triangles.  */
class IndexedMesh {
    var numTriangles = 0
    var indices: Any? = null // TODO not nullable?
    var triangleIndexBase = 0
    /** Size in byte of the indices for one triangle (3 * indexType.size if the indices are tightly packed)   */
    var triangleIndexStride = 0
    var numVertices = 0
    var vertices: Any? = null // TODO not nullable?
    var vertexBase = 0
    /** Size of a vertex, in bytes  */
    var vertexStride = 0
    /** The index type is set when adding an indexed mesh to the TriangleIndexVertexArray, do not set it manually   */
    var indexType = PHY_ScalarType.INTEGER
    /** The vertex type has a default type similar to Bullet's precision mode (float or double) but can be set manually
     *  if you for example run Bullet with double precision but have mesh data in single precision..    */
    var vertexType = PHY_ScalarType.FLOAT

    operator fun component1() = vertices!!
    operator fun component2() = vertexBase
    operator fun component3() = numVertices
    operator fun component4() = vertexType
    operator fun component5() = vertexStride
    operator fun component6() = indices!!
    operator fun component7() = triangleIndexBase
    operator fun component8() = triangleIndexStride
    operator fun component9() = numTriangles
    operator fun component10() = indexType
}

/** The TriangleIndexVertexArray allows to access multiple triangle meshes, by indexing into existing triangle/index arrays.
 *  Additional meshes can be added using addIndexedMesh
 *  No duplicate is made of the vertex/index data, it only indexes into external vertex/index arrays.
 *  So keep those arrays around during the lifetime of this TriangleIndexVertexArray.   */
class TriangleIndexVertexArray : StridingMeshInterface() {

    var indexedMeshes = ArrayList<IndexedMesh>()
    var hasAabb = false
    val aabbMin = Vec3()
    val aabbMax = Vec3()

    // just to be backwards compatible
//    btTriangleIndexVertexArray(int numTriangles, int * triangleIndexBase, int triangleIndexStride, int numVertices, btScalar * vertexBase, int vertexStride);

    fun addIndexedMesh(mesh: IndexedMesh, indexType: PHY_ScalarType = PHY_ScalarType.INTEGER) {
        indexedMeshes.add(mesh)
        indexedMeshes.last().indexType = indexType
    }

    /** JVM specific, returns the mesh with filled data */
    override fun getLockedVertexIndexBase(subPart: Int): IndexedMesh {
        assert(subPart < numSubParts)
        return indexedMeshes[subPart]
    }

    /** JVM specific, returns the mesh with filled data */
    override fun getLockedReadOnlyVertexIndexBase(subPart: Int) = indexedMeshes[subPart]

    /** unLockVertexBase finishes the access to a subpart of the triangle mesh
     *  make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished */
    override fun unLockVertexBase(subPart: Int) = Unit

    override fun unLockReadOnlyVertexBase(subPart: Int) = Unit

    /** numSubParts returns the number of seperate subparts, each subpart has a continuous array of vertices and indices    */
    override val numSubParts get() = indexedMeshes.size

    override fun preallocateVertices(numVerts: Int) = Unit
    override fun preallocateIndices(numIndices: Int) = Unit

    //virtual bool	hasPremadeAabb() const;
    override fun setPremadeAabb(aabbMin: Vec3, aabbMax: Vec3) {
        this.aabbMin put aabbMin    // TODO check if copy reference or not
        this.aabbMax put aabbMax
        hasAabb = true
    }
    override fun getPremadeAabb(aabbMin: Vec3, aabbMax: Vec3) {
        this.aabbMin put aabbMin
        this.aabbMax put aabbMax
    }
}