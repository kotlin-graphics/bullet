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

import bullet.linearMath.Vec3

// Don't change following order of parameters
class ConstraintRow {
    val normal = FloatArray(3)
    var rhs = 0f
    var jacDiagInv = 0f
    var lowerLimit = 0f
    var upperLimit = 0f
    var accumImpulse = 0f
}

enum class ContactPointFlags { LATERAL_FRICTION_INITIALIZED, HAS_CONTACT_CFM, HAS_CONTACT_ERP, CONTACT_STIFFNESS_DAMPING, FRICTION_ANCHOR;

    val i = 1 shl ordinal
}

infix fun Int.or(b: ContactPointFlags) = or(b.i)
infix fun Int.has(b: ContactPointFlags) = (this and b.i) != 0
infix fun Int.hasnt(b: ContactPointFlags) = (this and b.i) == 0

/** ManifoldContactPoint collects and maintains persistent contactpoints.
 *  used to improve stability and performance of rigidbody dynamics response.   */
class ManifoldPoint {


    val localPointA = Vec3()
    val localPointB = Vec3()
    val positionWorldOnB = Vec3()
    /** m_positionWorldOnA is redundant information, see getPositionWorldOnA(), but for clarity */
    val positionWorldOnA = Vec3()
    val normalWorldOnB = Vec3()

    var distance = 0f
    var combinedFriction = 0f
    /** torsional friction orthogonal to contact normal, useful to make spheres stop rolling forever    */
    var combinedRollingFriction = 0f
    /** torsional friction around contact normal, useful for grasping objects   */
    var combinedSpinningFriction = 0f
    var combinedRestitution = 0f

    //BP mod, store contact triangles.
    var partId0 = 0
    var partId1 = 0
    var index0 = 0
    var index1 = 0

    var userPersistentData: Any? = null
    var contactPointFlags = 0

    var appliedImpulse = 0f

    var appliedImpulseLateral1 = 0f
    var appliedImpulseLateral2 = 0f
    var contactMotion1 = 0f
    var contactMotion2 = 0f

    private var union0 = 0f

    var contactCFM
        get() = union0
        set(value) {
            union0 = value
        }
    var combinedContactStiffness1
        get() = union0
        set(value) {
            union0 = value
        }

    private var union1 = 0f
    var contactERP
        get() = union1
        set(value) {
            union1 = value
        }
    var combinedContactDamping1
        get() = union1
        set(value) {
            union1 = value
        }

    var frictionCFM = 0f
    /** lifetime of the contactpoint in frames  */
    var lifeTime = 0f

    val lateralFrictionDir1 = Vec3()
    val lateralFrictionDir2 = Vec3()


    constructor()
    constructor(pointA: Vec3, pointB: Vec3, normal: Vec3, distance: Float) {
        localPointA put pointA
        localPointB put pointB
        normalWorldOnB put normal
        this.distance = distance
    }
}