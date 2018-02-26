/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

package bullet

import bullet.collision.broadphaseCollision.DbvtBroadphase
import bullet.collision.collisionDispatch.CollisionDispatcher
import bullet.collision.collisionDispatch.DefaultCollisionConfiguration
import bullet.collision.collisionShapes.BoxShape
import bullet.collision.collisionShapes.CollisionShape
import bullet.collision.collisionShapes.SphereShape
import bullet.dynamics.constraintSolver.SequentialImpulseConstraintSolver
import bullet.dynamics.dynamics.DiscreteDynamicsWorld
import bullet.dynamics.dynamics.RigidBody
import bullet.linearMath.DefaultMotionState
import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import io.kotlintest.matchers.shouldBe
import io.kotlintest.specs.StringSpec

// This is a Hello World program for running a basic Bullet physics simulation

class helloWorld : StringSpec() {

    init {
        test()
    }
}

fun test() {

    // -----includes_end-----

    // -----initialization_start-----

    // collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
    val collisionConfiguration = DefaultCollisionConfiguration()

    // use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    val dispatcher = CollisionDispatcher(collisionConfiguration)

    // DbvtBroadphase is a good general purpose broadphase. You can also try out Axis3Sweep.
    val overlappingPairCache = DbvtBroadphase()

    // the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    val solver = SequentialImpulseConstraintSolver()

    val dynamicsWorld = DiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration)

    dynamicsWorld.gravity = Vec3(0, -10, 0)

    // -----initialization_end-----

    // keep track of the shapes, we release memory at exit.
    // make sure to re-use collision shapes among rigid bodies whenever possible!
    val collisionShapes = ArrayList<CollisionShape>()

    // create a few basic rigid bodies

    // the ground is a cube of side 100 at position y = -56.
    // the sphere will hit it at y = -6, with center at -5
    run {
        val groundShape = BoxShape(Vec3(50f))

        collisionShapes.add(groundShape)

        val groundTransform = Transform().apply {
            setIdentity()
            origin.put(0, -56, 0)
        }

        val mass = 0f

        // rigidbody is dynamic if and only if mass is non zero, otherwise static
        val isDynamic = mass != 0f

        val localInertia = Vec3()
        if (isDynamic)
            groundShape.calculateLocalInertia(mass, localInertia)

        // using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
        val myMotionState = DefaultMotionState(groundTransform)
        val rbInfo = RigidBody.RigidBodyConstructionInfo(mass, myMotionState, groundShape, localInertia)
        val body = RigidBody(rbInfo)

        // add the body to the dynamics world
        dynamicsWorld.addRigidBody(body)
    }

    // create a dynamic rigidbody
    run {
        // val colShape = BoxShape(Vec3(1))
        val colShape = SphereShape(1f)
        collisionShapes.add(colShape)

        // Create Dynamic Objects
        val startTransform = Transform().apply { setIdentity() }

        val mass = 1f

        // rigidbody is dynamic if and only if mass is non zero, otherwise static
        val isDynamic = mass != 0f

        val localInertia = Vec3()
        if (isDynamic)
            colShape.calculateLocalInertia(mass, localInertia)

        startTransform.origin.put(2, 10, 0)

        // using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
        val myMotionState = DefaultMotionState(startTransform)
        val rbInfo = RigidBody.RigidBodyConstructionInfo(mass, myMotionState, colShape, localInertia)
        val body = RigidBody(rbInfo)

        dynamicsWorld.addRigidBody(body)
    }

    // Do some simulation

    // -----stepsimulation_start-----
    for (i in 0..149) {

        dynamicsWorld.stepSimulation(1f / 60f, 10)

        // print positions of all objects
        for (j in dynamicsWorld.numCollisionObjects - 1 downTo 0) {

            val obj = dynamicsWorld.collisionObjects[j]
            val body = RigidBody.upcast(obj)
            val trans = body?.motionState?.run { Transform().also(::getWorldTransform) } ?: obj.getWorldTransform()
            if (j == 0)
                trans.origin shouldBe Vec3(0f, -56f, 0f)
            else
                println("[$i] world pos object $j = ${trans.origin.x}, ${trans.origin.y}, ${trans.origin.z}").also {
                    if (i == 149)
                        trans.origin shouldBe Vec3(2.000063f, -5.0000086f, 9.15244E-5f)
                }
        }
    }

    // -----stepsimulation_end-----

    // cleanup in the reverse order of creation/initialization

    // -----cleanup_start-----

    // remove the rigidbodies from the dynamics world and delete them
    for (i in dynamicsWorld.numCollisionObjects - 1 downTo 0) {
        val obj = dynamicsWorld.collisionObjects[i]
        dynamicsWorld.removeCollisionObject(obj)
    }

    // delete collision shapes
    collisionShapes.clear()
}
