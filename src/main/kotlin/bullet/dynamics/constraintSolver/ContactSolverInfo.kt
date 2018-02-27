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

enum class SolverMode {
    RANDOMIZE_ORDER,
    FRICTION_SEPARATE,
    USE_WARMSTARTING,
    USE_2_FRICTION_DIRECTIONS,
    ENABLE_FRICTION_DIRECTION_CACHING,
    DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION,
    CACHE_FRIENDLY,
    SIMD,
    INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS,
    ALLOW_ZERO_LENGTH_FRICTION_DIRECTIONS,
    DISABLE_IMPLICIT_CONE_FRICTION;

    val i = 1 shl ordinal
}

infix fun SolverMode.or(b: SolverMode) = i or b.i
infix fun Int.has(b: SolverMode) = (this and b.i) != 0
infix fun Int.hasnt(b: SolverMode) = (this and b.i) == 0

open class ContactSolverInfoData {

    var tau = 0f
    /** global non-contact constraint damping, can be locally overridden by constraints during 'getInfo2'.  */
    var damping = 0f
    var friction = 0f
    var timeStep = 0f
    var restitution = 0f
    var numIterations = 0
    var maxErrorReduction = 0f
    /** successive over-relaxation term */
    var sor = 0f
    /** error reduction for non-contact constraints */
    var erp = 0f
    /** error reduction for contact constraints */
    var erp2 = 0f
    /** constraint force mixing for contacts and non-contacts   */
    var globalCfm = 0f
    /** error reduction for friction constraints    */
    var frictionERP = 0f
    /** constraint force mixing for friction constraints    */
    var frictionCFM = 0f

    var splitImpulse = false
    var splitImpulsePenetrationThreshold = 0f
    var splitImpulseTurnErp = 0f
    var linearSlop = 0f
    var warmstartingFactor = 0f

    var solverMode = 0
    var restingContactRestitutionThreshold = 0
    var minimumSolverBatchSize = 0
    var maxGyroscopicForce = 0f
    var singleAxisRollingFrictionThreshold = 0f
    var leastSquaresResidualThreshold = 0f
    var restitutionVelocityThreshold = 0f
}

class ContactSolverInfo : ContactSolverInfoData() {
    init {
        tau = 0.6f
        damping = 1f
        friction = 0.3f
        timeStep = 1f / 60f
        restitution = 0f
        maxErrorReduction = 20f
        numIterations = 10
        erp = 0.2f
        erp2 = 0.2f
        globalCfm = 0f
        frictionERP = 0.2f  // positional friction 'anchors' are disabled by default
        frictionCFM = 0f
        sor = 1f
        splitImpulse = true
        splitImpulsePenetrationThreshold = -.04f
        splitImpulseTurnErp = 0.1f
        linearSlop = 0f
        warmstartingFactor = 0.85f
        //m_solverMode =  SOLVER_USE_WARMSTARTING |  SOLVER_SIMD | SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION|SOLVER_USE_2_FRICTION_DIRECTIONS|SOLVER_ENABLE_FRICTION_DIRECTION_CACHING;// | SOLVER_RANDMIZE_ORDER;
        solverMode = SolverMode.USE_WARMSTARTING or SolverMode.SIMD// | SOLVER_RANDMIZE_ORDER;
        restingContactRestitutionThreshold = 2//unused as of 2.81
        minimumSolverBatchSize = 128 //try to combine islands until the amount of constraints reaches this limit
        maxGyroscopicForce = 100f ///it is only used for 'explicit' version of gyroscopic force
        singleAxisRollingFrictionThreshold = 1e30f///if the velocity is above this threshold, it will use a single constraint row (axis), otherwise 3 rows.
        leastSquaresResidualThreshold = 0f
        restitutionVelocityThreshold = 0.2f//if the relative velocity is below this threshold, there is zero restitution
    }
}