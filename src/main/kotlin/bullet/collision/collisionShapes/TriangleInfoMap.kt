/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2010 Erwin Coumans  http://bulletphysics.org

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

import bullet.linearMath.PI2

// for btTriangleInfo m_flags
val TRI_INFO_V0V1_CONVEX = 1
val TRI_INFO_V1V2_CONVEX = 2
val TRI_INFO_V2V0_CONVEX = 4

val TRI_INFO_V0V1_SWAP_NORMALB = 8
val TRI_INFO_V1V2_SWAP_NORMALB = 16
val TRI_INFO_V2V0_SWAP_NORMALB = 32


/** The TriangleInfo structure stores information to adjust collision normals to avoid collisions against internal edges
 *  it can be generated using */
class TriangleInfo {
    var flags = 0
    var edgeV0V1Angle = PI2
    var edgeV1V2Angle = PI2
    var edgeV2V0Angle = PI2
}

/** The TriangleInfoMap stores edge angle information for some triangles. You can compute this information yourself or
 *  using GenerateInternalEdgeInfo. */
class TriangleInfoMap : HashMap<Int, TriangleInfo>() {
    /** used to determine if an edge or contact normal is convex, using the dot product */
    var convexEpsilon = 0f
    /** used to determine if a triangle edge is planar with zero angle */
    var planarEpsilon = 0.0001f
    /** used to compute connectivity: if the distance between two vertices is smaller than m_equalVertexThreshold,
     *  they are considered to be 'shared' */
    var equalVertexThreshold = 0.0001f * 0.0001f
    /** used to determine edge contacts: if the closest distance between a contact point and an edge is smaller than
     *  this distance threshold it is considered to "hit the edge"  */
    var edgeDistanceThreshold = 0.1f
    /** ignore edges that connect triangles at an angle larger than this maxEdgeAngleThreshold */
    var maxEdgeAngleThreshold = 0.0001f * 0.0001f
    /** used to determine if a triangle is degenerate (length squared of cross product of 2 triangle edges < threshold) */
    var zeroAreaThreshold = PI2
}