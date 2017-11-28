/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

package bullet.linearMath

import bullet.has
import bullet.hasnt
import kotlin.math.max
import kotlin.math.min
import kotlin.reflect.KMutableProperty0

fun aabbExpand(aabbMin: Vec3, aabbMax: Vec3, expansionMin: Vec3, expansionMax: Vec3) {
    aabbMin put aabbMin + expansionMin
    aabbMax put aabbMax + expansionMax
}

/** conservative test for overlap between two aabbs */
fun testPointAgainstAabb2(aabbMin1: Vec3, aabbMax1: Vec3, point: Vec3): Boolean {
    var overlap = true
    overlap = if (aabbMin1.x > point.x || aabbMax1.x < point.x) false else overlap
    overlap = if (aabbMin1.z > point.z || aabbMax1.z < point.z) false else overlap
    overlap = if (aabbMin1.y > point.y || aabbMax1.y < point.y) false else overlap
    return overlap
}


/** conservative test for overlap between two aabbs */
fun testAabbAgainstAabb2(aabbMin1: Vec3, aabbMax1: Vec3, aabbMin2: Vec3, aabbMax2: Vec3): Boolean {
    var overlap = true
    overlap = if (aabbMin1.x > aabbMax2.x || aabbMax1.x < aabbMin2.x) false else overlap
    overlap = if (aabbMin1.z > aabbMax2.z || aabbMax1.z < aabbMin2.z) false else overlap
    overlap = if (aabbMin1.y > aabbMax2.y || aabbMax1.y < aabbMin2.y) false else overlap
    return overlap
}

/** conservative test for overlap between triangle and aabb */
fun testTriangleAgainstAabb2(vertices: Array<Vec3>, aabbMin: Vec3, aabbMax: Vec3): Boolean {
    val (p1, p2, p3) = vertices
    return when {
        min(min(p1[0], p2[0]), p3[0]) > aabbMax[0] || max(max(p1[0], p2[0]), p3[0]) < aabbMin[0] -> false
        min(min(p1[2], p2[2]), p3[2]) > aabbMax[2] || max(max(p1[2], p2[2]), p3[2]) < aabbMin[2] -> false
        min(min(p1[1], p2[1]), p3[1]) > aabbMax[1] || max(max(p1[1], p2[1]), p3[1]) < aabbMin[1] -> false
        else -> true
    }
}

fun outcode(p: Vec3, halfExtent: Vec3) = (if (p.x < -halfExtent.x) 0x01 else 0x0) or (if (p.x > halfExtent.x) 0x08 else 0x0) or
        (if (p.y < -halfExtent.y) 0x02 else 0x0) or (if (p.y > halfExtent.y) 0x10 else 0x0) or
        (if (p.z < -halfExtent.z) 0x4 else 0x0) or (if (p.z > halfExtent.z) 0x20 else 0x0)

fun rayAabb2(rayFrom: Vec3, rayInvDirection: Vec3, raySign: IntArray, bounds: Array<Vec3>, tMin: KMutableProperty0<Float>,
             lambdaMin: Float, lambdaMax: Float): Boolean {

    tMin.set((bounds[raySign[0]].x - rayFrom.x) * rayInvDirection.x)
    var tMax = (bounds[1 - raySign[0]].x - rayFrom.x) * rayInvDirection.x
    val tYmin = (bounds[raySign[1]].y - rayFrom.y) * rayInvDirection.y
    val tYmax = (bounds[1 - raySign[1]].y - rayFrom.y) * rayInvDirection.y

    if (tMin() > tYmax || tYmin > tMax) return false

    if (tYmin > tMin()) tMin.set(tYmin)

    if (tYmax < tMax) tMax = tYmax

    val tZmin = (bounds[raySign[2]].z - rayFrom.z) * rayInvDirection.z
    val tZmax = (bounds[1 - raySign[2]].z - rayFrom.z) * rayInvDirection.z

    if (tMin() > tZmax || tZmin > tMax) return false
    if (tZmin > tMin()) tMin.set(tZmin)
    if (tZmax < tMax) tMax = tZmax
    return (tMin() < lambdaMax && tMax > lambdaMin)
}

fun rayAabb(rayFrom: Vec3, rayTo: Vec3, aabbMin: Vec3, aabbMax: Vec3, param: KMutableProperty0<Float>, normal: Vec3): Boolean {
    val aabbHalfExtent = (aabbMax - aabbMin) * 0.5f
    val aabbCenter = (aabbMax + aabbMin) * 0.5f
    val source = rayFrom - aabbCenter
    val target = rayTo - aabbCenter
    val sourceOutcode = outcode(source, aabbHalfExtent)
    val targetOutcode = outcode(target, aabbHalfExtent)
    if (sourceOutcode hasnt targetOutcode) {
        var lambdaEnter = 0f
        var lambdaExit = param()
        val r = target - source
        var normSign = 1f
        val hitNormal = Vec3()
        var bit = 1

        for (j in 0..1) {
            var i = 0
            while (i != 3) {
                if (sourceOutcode has bit) {
                    val lambda = (-source[i] - aabbHalfExtent[i] * normSign) / r[i]
                    if (lambdaEnter <= lambda) {
                        lambdaEnter = lambda
                        hitNormal put 0f
                        hitNormal[i] = normSign
                    }
                } else if (targetOutcode has bit) {
                    val lambda = (-source[i] - aabbHalfExtent[i] * normSign) / r[i]
                    if (lambda < lambdaExit) lambdaExit = lambda
                }
                bit = bit shl 1
                ++i
            }
            normSign = -1f
        }
        if (lambdaEnter <= lambdaExit) {
            param.set(lambdaEnter)
            normal put hitNormal
            return true
        }
    }
    return false
}

fun transformAabb(halfExtents: Vec3, margin: Float, t: Transform, aabbMinOut: Vec3, aabbMaxOut: Vec3) {
    val halfExtentsWithMargin = halfExtents + margin
    val abs_b = t.basis.absolute()
    val center = t.origin
    val extent = halfExtentsWithMargin dot3 abs_b
    aabbMinOut put center - extent
    aabbMaxOut put center + extent
}


fun transformAabb(localAabbMin: Vec3, localAabbMax: Vec3, margin: Float, trans: Transform, aabbMinOut: Vec3, aabbMaxOut: Vec3) {
    assert(localAabbMin.x <= localAabbMax.x)
    assert(localAabbMin.y <= localAabbMax.y)
    assert(localAabbMin.z <= localAabbMax.z)
    val localHalfExtents = 0.5f * (localAabbMax - localAabbMin)
    localHalfExtents += Vec3(margin)

    val localCenter = 0.5f * (localAabbMax + localAabbMin)
    val abs_b = trans.basis.absolute()
    val center = trans * localCenter
    val extent = localHalfExtents dot3 abs_b
    aabbMinOut put (center - extent)
    aabbMaxOut put (center + extent)
}