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

package bullet.collision.collisionDispatch

import bullet.EPSILON
import bullet.collision.collisionShapes.SphereShape
import bullet.collision.collisionShapes.TriangleShape
import bullet.collision.narrowPhaseCollision.DiscreteCollisionDetectorInterface
import bullet.linearMath.DebugDraw
import bullet.linearMath.Vec3
import bullet.linearMath.times
import kotlin.math.sqrt

/** sphere-triangle to match the DiscreteCollisionDetectorInterface */
class SphereTriangleDetector(val sphere: SphereShape, val triangle: TriangleShape, val contactBreakingThreshold: Float) :
        DiscreteCollisionDetectorInterface {

    override fun getClosestPoints(input: DiscreteCollisionDetectorInterface.ClosestPointInput, output: DiscreteCollisionDetectorInterface.Result, debugDraw: DebugDraw?, swapResults: Boolean) {

        val transformA = input.transformA
        val transformB = input.transformB

        val point = Vec3()
        val normal = Vec3()
        //move sphere into triangle space
        val sphereInTr = transformB.inverseTimes(transformA)

        val (res, depth) = collide(sphereInTr.origin, point, normal, contactBreakingThreshold)
        if (res)
            if (swapResults) {
                val normalOnB = transformB.basis * normal
                val normalOnA = -normalOnB
                val pointOnA = transformB * point + normalOnB * depth
                output.addContactPoint(normalOnA, pointOnA, depth)
            } else
                output.addContactPoint(transformB.basis * normal, transformB * point, depth)
    }
//    virtual void    getClosestPoints(const ClosestPointInput& input,Result& output,
//    class btIDebugDraw* debugDraw,bool swapResults=false)

    fun collide(sphereCenter: Vec3, point: Vec3, resultNormal: Vec3, contactBreakingThreshold: Float): Pair<Boolean, Float> {

        val vertices = triangle.vertices

        val radius = sphere.radius
        val radiusWithThreshold = radius + contactBreakingThreshold

        val normal = (vertices[1] - vertices[0]) cross (vertices[2] - vertices[0])

        val l2 = normal.length2()
        var hasContact = false
        val contactPoint = Vec3()

        if (l2 >= Float.EPSILON * Float.EPSILON) {
            normal /= sqrt(l2)

            val p1ToCentre = sphereCenter - vertices[0]
            var distanceFromPlane = p1ToCentre dot normal

            if (distanceFromPlane < 0f) {
                //triangle facing the other way
                distanceFromPlane *= -1f
                normal *= -1f
            }

            val isInsideContactPlane = distanceFromPlane < radiusWithThreshold

            // Check for contact / intersection
            if (isInsideContactPlane)
                if (facecontains(sphereCenter, vertices, normal)) {
                    // Inside the contact wedge - touches a point on the shell plane
                    hasContact = true
                    contactPoint put sphereCenter - normal * distanceFromPlane
                } else {
                    // Could be inside one of the contact capsules
                    val contactCapsuleRadiusSqr = radiusWithThreshold * radiusWithThreshold
                    val nearestOnEdge = Vec3()
                    for (i in 0 until triangle.numEdges) {
                        val pa = Vec3()
                        val pb = Vec3()

                        triangle.getEdge(i, pa, pb)

                        val distanceSqr = segmentSqrDistance(pa, pb, sphereCenter, nearestOnEdge)
                        if (distanceSqr < contactCapsuleRadiusSqr) {
                            // Yep, we're inside a capsule
                            hasContact = true
                            contactPoint put nearestOnEdge
                        }
                    }
                }
        }

        var depth = 0f

        if (hasContact) {
            val contactToCentre = sphereCenter - contactPoint
            val distanceSqr = contactToCentre.length2()

            if (distanceSqr < radiusWithThreshold * radiusWithThreshold) {
                if (distanceSqr > Float.EPSILON) {
                    val distance = sqrt(distanceSqr)
                    resultNormal put contactToCentre
                    resultNormal.normalize()
                    point put contactPoint
                    depth = -(radius - distance)
                } else {
                    resultNormal put normal
                    point put contactPoint
                    depth = -radius
                }
                return true to depth
            }
        }
        return false to depth
    }

    /** See also geometrictools.com
     *  Basic idea: D = |p - (lo + t0*lv)| where t0 = lv . (p - lo) / lv . lv   */
    fun segmentSqrDistance(from: Vec3, to: Vec3, p: Vec3, nearest: Vec3): Float {
        val diff = p - from
        val v = to - from
        var t = v dot diff

        if (t > 0) {
            val dotVV = v dot v
            if (t < dotVV) {
                t /= dotVV
                diff -= t * v
            } else {
                t = 1f
                diff -= v
            }
        } else t = 0f

        nearest put from + t * v // TODO check
        return diff dot diff
    }

    fun pointInTriangle(vertices: Array<Vec3>, normal: Vec3, p: Vec3): Boolean {
        val (p1, p2, p3) = vertices

        val edge1 = p2 - p1
        val edge2 = p3 - p2
        val edge3 = p1 - p3

        val p1_to_p = p - p1
        val p2_to_p = p - p2
        val p3_to_p = p - p3

        val edge1_normal = edge1 cross normal
        val edge2_normal = edge2 cross normal
        val edge3_normal = edge3 cross normal

        val r1 = edge1_normal dot p1_to_p
        val r2 = edge2_normal dot p2_to_p
        val r3 = edge3_normal dot p3_to_p
        return (r1 > 0 && r2 > 0 && r3 > 0) || (r1 <= 0 && r2 <= 0 && r3 <= 0)
    }

    fun facecontains(p: Vec3, vertices: Array<Vec3>, normal: Vec3) = pointInTriangle(vertices, Vec3(normal), Vec3(p))
//
//    btSphereShape* m_sphere
//    btTriangleShape* m_triangle
//    btScalar    m_contactBreakingThreshold
}