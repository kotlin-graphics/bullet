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

package bullet.dynamics.dynamics

import bullet.collision.broadphaseCollision.BroadphaseInterface
import bullet.collision.broadphaseCollision.Dispatcher
import bullet.collision.collisionDispatch.CollisionConfiguration
import bullet.collision.collisionDispatch.CollisionWorld
import bullet.dynamics.constraintSolver.ConstraintSolver
import bullet.dynamics.constraintSolver.ContactSolverInfo
import bullet.dynamics.constraintSolver.TypedConstraint
import bullet.linearMath.Vec3

/** Type for the callback for each tick */
typealias InternalTickCallback = (DynamicsWorld, Float) -> Unit

enum class DynamicsWorldType { SIMPLE_DYNAMICS_WORLD, DISCRETE_DYNAMICS_WORLD, CONTINUOUS_DYNAMICS_WORLD,
    SOFT_RIGID_DYNAMICS_WORLD, GPU_DYNAMICS_WORLD, SOFT_MULTIBODY_DYNAMICS_WORLD;

    val i = ordinal + 1
}

/** The DynamicsWorld is the interface class for several dynamics implementation, basic, discrete, parallel, and
 *  continuous etc. */
abstract class DynamicsWorld(dispatcher: Dispatcher?, broadphase: BroadphaseInterface, collisionConfiguration: CollisionConfiguration)
    : CollisionWorld(dispatcher, broadphase, collisionConfiguration) {

    var internalTickCallback: InternalTickCallback? = null
    var internalPreTickCallback: InternalTickCallback? = null
    var worldUserInfo: Any? = null

    var solverInfo = ContactSolverInfo()

    /** stepSimulation proceeds the simulation over 'timeStep', units in preferably in seconds.
     *  By default, Bullet will subdivide the timestep in constant substeps of each 'fixedTimeStep'.
     *  in order to keep the simulation real-time, the maximum number of substeps can be clamped to 'maxSubSteps'.
     *  You can disable subdividing the timestep/substepping by passing maxSubSteps = 0 as second argument to stepSimulation,
     *  but in that case you have to keep the timeStep constant. */
    abstract fun stepSimulation(timeStep: Float, maxSubSteps: Int = 1, fixedTimeStep: Float = 1f / 60f): Int

    abstract override fun debugDrawWorld()

    abstract fun addConstraint(constraint: TypedConstraint, disableCollisionsBetweenLinkedBodies: Boolean = false)

    abstract fun removeConstraint(constraint: TypedConstraint)

    abstract fun addAction(action: ActionInterface)

    abstract fun removeAction(action: ActionInterface)

    /** once a rigidbody is added to the dynamics world, it will get this gravity assigned existing rigidbodies in the
     *  world get gravity assigned too, during this method */
    abstract var gravity: Vec3

    abstract fun synchronizeMotionStates()

    abstract fun addRigidBody(body: RigidBody)

    abstract fun addRigidBody(body: RigidBody, group: Int, mask: Int)

    abstract fun removeRigidBody(body: RigidBody)

    abstract var constraintSolver: ConstraintSolver

    open val numConstraints get() = 0

    open fun getConstraint(index: Int): TypedConstraint? = null

    abstract val worldType: DynamicsWorldType

    abstract fun clearForces()

    /** Set the callback for when an internal tick (simulation substep) happens, optional user info */
    fun setInternalTickCallback(cb: InternalTickCallback, worldUserInfo: Any? = null, isPreTick: Boolean = false) {
        if (isPreTick)
            internalPreTickCallback = cb
        else
            internalTickCallback = cb
        this.worldUserInfo = worldUserInfo
    }
}