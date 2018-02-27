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

import bullet.collision.collisionDispatch.CollisionObject
import bullet.collision.collisionDispatch.islandId
import bullet.has
import bullet.linearMath.Transform
import bullet.linearMath.TypedObject
import bullet.linearMath.Vec3
import bullet.linearMath.Vec4
import kotlin.math.max
import bullet.collision.narrowPhaseCollision.ContactPointFlags as CPF

//the enum starts at 1024 to avoid type conflicts with TypedConstraint
val BT_PERSISTENT_MANIFOLD_TYPE = 1024 + 1

val MANIFOLD_CACHE_SIZE = 4

typealias ContactDestroyedCallback = (Any) -> Boolean
typealias ContactProcessedCallback = (ManifoldPoint, Any?, Any?) -> Boolean
typealias ContactStartedCallback = (PersistentManifold) -> Unit
typealias ContactEndedCallback = (PersistentManifold) -> Unit

var gContactBreakingThreshold = 0.02f
var gContactDestroyedCallback: ContactDestroyedCallback? = null
var gContactProcessedCallback: ContactProcessedCallback? = null
var gContactStartedCallback: ContactStartedCallback? = null
var gContactEndedCallback: ContactEndedCallback? = null

/** contactCalcArea3Points will approximate the convex hull area using 3 points
 *  when setting it to false, it will use 4 points to compute the area: it is more accurate but slower  */
var contactCalcArea3Points = true


/** PersistentManifold is a contact point cache, it stays persistent as long as objects are overlapping in the broadphase.
 *  Those contact points are created by the collision narrow phase.
 *  The cache can be empty, or hold 1, 2, 3 or 4 points. Some collision algorithms (GJK) might only add one point at a time.
 *  updates/refreshes old contact points, and throw them away if necessary (distance becomes too large)
 *  reduces the cache to 4 points, when more then 4 points are added, using following rules:
 *  the contact point with deepest penetration is always kept, and it tries to maximuze the area covered by the points
 *  note that some pairs of objects might have more then one contact manifold.  */
class PersistentManifold : TypedObject {

    private val pointCache = Array(MANIFOLD_CACHE_SIZE, { ManifoldPoint() })

    /// this two body pointers can point to the physics rigidbody class.
    var body0: CollisionObject? = null
    var body1: CollisionObject? = null

    private var cachedPoints = 0
    /** @todo: get this margin from the current physics / collision environment */
    var contactBreakingThreshold = 0f
    var contactProcessingThreshold = 0f

    var companionIdA = 0
    var companionIdB = 0

    var index1a = 0

    constructor() : super(BT_PERSISTENT_MANIFOLD_TYPE)
    constructor(body0: CollisionObject, body1: CollisionObject, wtf: Int, contactBreakingThreshold: Float,
                contactProcessingThreshold: Float) : super(BT_PERSISTENT_MANIFOLD_TYPE) {
        this.body0 = body0
        this.body1 = body1
        this.contactBreakingThreshold = contactBreakingThreshold
        this.contactProcessingThreshold = contactProcessingThreshold
    }

    /** sort cached points so most isolated points come first   */
    fun sortCachedPoints(pt: ManifoldPoint): Int {
        // calculate 4 possible cases areas, and take biggest area. Also need to keep 'deepest'
        var maxPenetrationIndex = -1
        var maxPenetration = pt.distance
        for (i in 0..3) {
            if (pointCache[i].distance < maxPenetration) {
                maxPenetrationIndex = i
                maxPenetration = pointCache[i].distance
            }
        }

        var res0 = 0f
        var res1 = 0f
        var res2 = 0f
        var res3 = 0f

        if (contactCalcArea3Points) {
            if (maxPenetrationIndex != 0) {
                val a0 = pt.localPointA - pointCache[1].localPointA
                val b0 = pointCache[3].localPointA - pointCache[2].localPointA
                val cross = a0 cross b0
                res0 = cross.length2()
            }
            if (maxPenetrationIndex != 1) {
                val a1 = pt.localPointA - pointCache[0].localPointA
                val b1 = pointCache[3].localPointA - pointCache[2].localPointA
                val cross = a1 cross b1
                res1 = cross.length2()
            }
            if (maxPenetrationIndex != 2) {
                val a2 = pt.localPointA - pointCache[0].localPointA
                val b2 = pointCache[3].localPointA - pointCache[1].localPointA
                val cross = a2 cross b2
                res2 = cross.length2()
            }
            if (maxPenetrationIndex != 3) {
                val a3 = pt.localPointA - pointCache[0].localPointA
                val b3 = pointCache[2].localPointA - pointCache[1].localPointA
                val cross = a3 cross b3
                res3 = cross.length2()
            }
        } else {
            if (maxPenetrationIndex != 0)
                res0 = calcArea4Points(pt.localPointA, pointCache[1].localPointA, pointCache[2].localPointA, pointCache[3].localPointA)
            if (maxPenetrationIndex != 1)
                res1 = calcArea4Points(pt.localPointA, pointCache[0].localPointA, pointCache[2].localPointA, pointCache[3].localPointA)
            if (maxPenetrationIndex != 2)
                res2 = calcArea4Points(pt.localPointA, pointCache[0].localPointA, pointCache[1].localPointA, pointCache[3].localPointA)
            if (maxPenetrationIndex != 3)
                res3 = calcArea4Points(pt.localPointA, pointCache[0].localPointA, pointCache[1].localPointA, pointCache[2].localPointA)
        }
        return Vec4(res0, res1, res2, res3).closestAxis4() // biggest area
    }

    // TODO
    fun findContactPoint(unUsed: Array<ManifoldPoint>, numUnused: Int, pt: ManifoldPoint) = 0

    fun setBodies(body0: CollisionObject, body1: CollisionObject) {
        this.body0 = body0
        this.body1 = body1
    }

    fun clearUserCache(pt: ManifoldPoint) {
        val oldPtr = pt.userPersistentData
        oldPtr?.let {
            gContactDestroyedCallback?.let {
                it(oldPtr)
                pt.userPersistentData = null
            }
        }
    }

    /** the setNumContacts API is usually not used, except when you gather/fill all contacts manually   */
    var numContacts
        get() = cachedPoints
        set(value) {
            cachedPoints = value
        }

    fun getContactPoint(index: Int): ManifoldPoint {
        assert(index < cachedPoints)
        return pointCache[index]
    }

    fun getCacheEntry(newPoint: ManifoldPoint): Int {
        var shortestDist = contactBreakingThreshold * contactBreakingThreshold
        val size = numContacts
        var nearestPoint = -1
        pointCache.forEachIndexed { i, mp ->
            val diffA = mp.localPointA - newPoint.localPointA
            val distToManiPoint = diffA dot diffA
            if (distToManiPoint < shortestDist) {
                shortestDist = distToManiPoint
                nearestPoint = i
            }
        }
        return nearestPoint
    }

    fun addManifoldPoint(newPoint: ManifoldPoint, isPredictive: Boolean = false): Int {
        if (!isPredictive)
            assert(validContactDistance(newPoint))
        var insertIndex = numContacts
        if (insertIndex == MANIFOLD_CACHE_SIZE) {
            if (MANIFOLD_CACHE_SIZE >= 4)
            //sort cache so best points come first, based on area
                insertIndex = sortCachedPoints(newPoint)
            else
                insertIndex = 0
            clearUserCache(pointCache[insertIndex])
        } else
            cachedPoints++
        if (insertIndex < 0) insertIndex = 0
        assert(pointCache[insertIndex].userPersistentData == null)
        pointCache[insertIndex] = newPoint
        return insertIndex
    }

    fun removeContactPoint(index: Int) {
        clearUserCache(pointCache[index])

        val lastUsedIndex = numContacts - 1
//		m_pointCache[index] = m_pointCache[lastUsedIndex];
        if (index != lastUsedIndex) {
            pointCache[index] = pointCache[lastUsedIndex]
            //get rid of duplicated userPersistentData pointer
            with(pointCache[lastUsedIndex]) {
                userPersistentData = 0
                appliedImpulse = 0f
                contactPointFlags = 0
                appliedImpulseLateral1 = 0f
                appliedImpulseLateral2 = 0f
                lifeTime = 0f
            }
        }
        assert(pointCache[lastUsedIndex].userPersistentData == null)
        cachedPoints--

        if (cachedPoints == 0) gContactEndedCallback?.invoke(this)
    }

    fun replaceContactPoint(newPoint: ManifoldPoint, insertIndex: Int) {
        assert(validContactDistance(newPoint))

        val lifeTime = pointCache[insertIndex].lifeTime
        val appliedImpulse = pointCache[insertIndex].appliedImpulse
        val appliedLateralImpulse1 = pointCache[insertIndex].appliedImpulseLateral1
        val appliedLateralImpulse2 = pointCache[insertIndex].appliedImpulseLateral2

        var replacePoint = true
        // we keep existing contact points for friction anchors if the friction force is within the Coulomb friction cone
        if (newPoint.contactPointFlags has CPF.FRICTION_ANCHOR.i) {
            //   printf("appliedImpulse=%f\n", appliedImpulse);
            //   printf("appliedLateralImpulse1=%f\n", appliedLateralImpulse1);
            //   printf("appliedLateralImpulse2=%f\n", appliedLateralImpulse2);
            //   printf("mu = %f\n", m_pointCache[insertIndex].m_combinedFriction);
            val mu = pointCache[insertIndex].combinedFriction
            val eps = 0  //we could allow to enlarge or shrink the tolerance to check against the friction cone a bit, say 1e-7
            val a = appliedLateralImpulse1 * appliedLateralImpulse1 + appliedLateralImpulse2 * appliedLateralImpulse2
            var b = eps + mu * appliedImpulse
            b = b * b
            replacePoint = a > b
        }
        if (replacePoint) {
            assert(lifeTime >= 0)
            val cache = pointCache[insertIndex].userPersistentData

            pointCache[insertIndex] = newPoint
            pointCache[insertIndex].userPersistentData = cache
            pointCache[insertIndex].appliedImpulse = appliedImpulse
            pointCache[insertIndex].appliedImpulseLateral1 = appliedLateralImpulse1
            pointCache[insertIndex].appliedImpulseLateral2 = appliedLateralImpulse2
        }
        pointCache[insertIndex].lifeTime = lifeTime
    }

    fun validContactDistance(pt: ManifoldPoint) = pt.distance <= contactBreakingThreshold

    /** calculated new worldspace coordinates and depth, and reject points that exceed the collision margin */
    fun refreshContactPoints(trA: Transform, trB: Transform) {

        var i = numContacts - 1
        /// first refresh worldspace positions and distance
        while (i >= 0) {
            val mp = pointCache[i]
            mp.positionWorldOnA put trA * mp.localPointA
            mp.positionWorldOnB put trB * mp.localPointB
            mp.distance = (mp.positionWorldOnA - mp.positionWorldOnB) dot mp.normalWorldOnB
            mp.lifeTime++
            i--
        }
        // then
        val projectedDifference = Vec3()
        val projectedPoint = Vec3()
        i = numContacts - 1
        while (i >= 0) {
            val mp = pointCache[i]
            //contact becomes invalid when signed distance exceeds margin (projected on contactnormal direction)
            if (!validContactDistance(mp))
                removeContactPoint(i)
            else {
                /*  todo: friction anchor may require the contact to be around a bit longer
                    contact also becomes invalid when relative movement orthogonal to normal exceeds margin                 */
                projectedPoint put mp.positionWorldOnA - mp.normalWorldOnB * mp.distance
                projectedDifference put mp.positionWorldOnB - projectedPoint
                val distance2d = projectedDifference dot projectedDifference
                if (distance2d > contactBreakingThreshold * contactBreakingThreshold)
                    removeContactPoint(i)
                else gContactProcessedCallback?.invoke(mp, body0, body1) //contact point processed callback
            }
            i--
        }
    }

    fun clearManifold() {
        for (i in 0 until cachedPoints) clearUserCache(pointCache[i])
        if (cachedPoints != 0) gContactEndedCallback?.invoke(this)
        cachedPoints = 0
    }
}

fun calcArea4Points(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3): Float {
    // It calculates possible 3 area constructed from random 4 points and returns the biggest one.

    val a = Array(3, { Vec3() })
    val b = Array(3, { Vec3() })
    a[0] = p0 - p1
    a[1] = p0 - p2
    a[2] = p0 - p3
    b[0] = p2 - p3
    b[1] = p1 - p3
    b[2] = p1 - p2

    //todo: Following 3 cross production can be easily optimized by SIMD.
    val tmp0 = a[0] cross b[0]
    val tmp1 = a[1] cross b[1]
    val tmp2 = a[2] cross b[2]

    return max(max(tmp0.length2(), tmp1.length2()), tmp2.length2())
}