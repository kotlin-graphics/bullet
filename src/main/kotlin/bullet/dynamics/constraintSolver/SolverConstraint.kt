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

package bullet.dynamics.constraintSolver

import bullet.BYTES
import bullet.collision.narrowPhaseCollision.ManifoldPoint
import bullet.linearMath.Vec3

/** 1D constraint along a normal axis between bodyA and bodyB. It can be combined to solve contact and friction constraints.    */
class SolverConstraint {

    val relpos1CrossNormal = Vec3()
    val contactNormal1 = Vec3()

    val relpos2CrossNormal = Vec3()
    /** usually m_contactNormal2 == -m_contactNormal1, but not always   */
    val contactNormal2 = Vec3()

    val angularComponentA = Vec3()
    val angularComponentB = Vec3()

    var appliedPushImpulse = 0f
    var appliedImpulse = 0f

    var friction = 0f
    var jacDiagABInv = 0f
    var rhs = 0f
    var cfm = 0f

    var lowerLimit = 0f
    var upperLimit = 0f
    var rhsPenetration = 0f

    private var union: Any? = null
    var originalContactPoint
        get() = union
        set(value) {
            union = value
        }
    var unusedPadding4
        get() = union as? Float
        set(value) {
            union = value
        }
    var numRowsForNonContactConstraint
        get() = union as? Int
        set(value) {
            union = value
        }

    var overrideNumSolverIterations = 0
    var frictionIndex = 0
    var solverBodyIdA = 0
    var solverBodyIdB = 0


    enum class SolverConstraintType { CONTACT_1D, FRICTION_1D;

        val i = ordinal
    }

    companion object {
        val size = 6 * Vec3.size + 14 * Float.BYTES
    }
}