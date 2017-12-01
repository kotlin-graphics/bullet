/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

package bullet.collision.narrowPhaseCollision

import bullet.collision.collisionShapes.ConvexShape
import bullet.collision.collisionShapes.TriangleCallback
import bullet.collision.collisionShapes.TriangleShape
import bullet.has
import bullet.hasnt
import bullet.linearMath.Transform
import bullet.linearMath.Vec3

open class TriangleRaycastCallback(val from: Vec3, val to: Vec3, flags: Int = 0) : TriangleCallback {

    //@BP Mod - allow backface filtering and unflipped normals
    enum class Flags(val i: Int) {
        None(0),
        FilterBackfaces(1 shl 0),
        /** Prevents returned face normal getting flipped when a ray hits a back-facing triangle    */
        KeepUnflippedNormal(1 shl 1),
        /** SubSimplexConvexCastRaytest is the default, even if kF_None is set. */
        /** Uses an approximate but faster ray versus convex intersection algorithm */
        UseSubSimplexConvexCastRaytest(1 shl 2),
        UseGjkConvexCastRaytest(1 shl 3),
        Terminator(0.inv())
    }

    var flags = 0

    var hitFraction = 1f

    override fun processTriangle(triangle: Array<Vec3>, partId: Int, triangleIndex: Int) {

        val (vert0, vert1, vert2) = triangle

        val v10 = vert1 - vert0
        val v20 = vert2 - vert0

        val triangleNormal = v10 cross v20

        val dist = vert0 dot triangleNormal
        val distA = (triangleNormal dot from) - dist
        val distB = (triangleNormal dot to) - dist

        if (distA * distB >= 0f) return // same sign

        if (flags has Flags.FilterBackfaces.i && distA <= 0f) return // Backface, skip check

        val projLength = distA - distB
        val distance = distA / projLength
        /*  Now we have the intersection point on the plane, we'll see if it's inside the triangle
            Add an epsilon as a tolerance for the raycast,
            In case the ray hits exacly on the edge of the triangle.
            It must be scaled for the triangle size.    */
        if (distance < hitFraction) {
            val edgeTolerance = triangleNormal.length2() * -0.0001f
            val point = Vec3().apply { setInterpolate3(from, to, distance) }
            val v0p = vert0 - point
            val v1p = vert1 - point
            val cp0 = v0p cross v1p
            if (cp0 dot triangleNormal >= edgeTolerance) {
                val v2p = vert2 - point
                val cp1 = v1p cross v2p
                if (cp1 dot triangleNormal >= edgeTolerance) {
                    val cp2 = v2p cross v0p
                    if (cp2 dot triangleNormal >= edgeTolerance) {
                        //@BP Mod
                        // Triangle normal isn't normalized
                        triangleNormal.normalize()
                        //@BP Mod - Allow for unflipped normal when raycasting against backfaces
                        hitFraction = reportHit(
                                if (flags hasnt Flags.KeepUnflippedNormal.i && distA <= 0f) -triangleNormal else triangleNormal,
                                distance, partId, triangleIndex)
                    }
                }
            }
        }
    }

    open fun reportHit(hitNormalLocal: Vec3, hitFraction: Float, partId: Int, triangleIndex: Int) = 0f
}

open class TriangleConvexcastCallback(val convexShape: ConvexShape, val convexShapeFrom: Transform, val convexShapeTo: Transform,
                                 val triangleToWorld: Transform, var triangleCollisionMargin: Float) : TriangleCallback {
    var hitFraction = 1f
    var allowedPenetration = 0f

    override fun processTriangle(triangle: Array<Vec3>, partId: Int, triangleIndex: Int) {

        val triangleShape = TriangleShape(triangle)
        triangleShape.margin = triangleCollisionMargin

        val simplexSolver = VoronoiSimplexSolver()
        val gjkEpaPenetrationSolver = GjkEpaPenetrationDepthSolver()

        val convexCaster = ContinuousConvexCollision(convexShape, triangleShape, simplexSolver, gjkEpaPenetrationSolver)

        val castResult = ConvexCast.CastResult().apply {
            fraction = 1f
            this.allowedPenetration = allowedPenetration
        }
        if (convexCaster.calcTimeOfImpact(convexShapeFrom, convexShapeTo, triangleToWorld, triangleToWorld, castResult))
        //add hit
            if (castResult.normal.length2() > 0.0001f)
                if (castResult.fraction < hitFraction) {
                    castResult.normal.normalize()
                    reportHit(castResult.normal, castResult.hitPoint, castResult.fraction, partId, triangleIndex)
                }
    }

    open fun reportHit(hitNormalLocal: Vec3, hitPointLocal: Vec3, hitFraction: Float, partId: Int, triangleIndex: Int) = 0f
}