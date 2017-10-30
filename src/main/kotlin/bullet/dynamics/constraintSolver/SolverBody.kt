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

import bullet.dynamics.dynamics.RigidBody
import bullet.linearMath.Transform
import bullet.linearMath.TransformUtil
import bullet.linearMath.Vec3
import bullet.linearMath.times

/** The SolverBody is an internal datastructure for the constraint solver. Only necessary data is packed to increase
 *  cache coherence/performance.    */
class SolverBody {

    var worldTransform = Transform()
    val deltaLinearVelocity = Vec3()
    val deltaAngularVelocity = Vec3()
    val angularFactor = Vec3()
    val linearFactor = Vec3()
    val invMass = Vec3()
    val pushVelocity = Vec3()
    val turnVelocity = Vec3()
    val linearVelocity = Vec3()
    val angularVelocity = Vec3()
    val externalForceImpulse = Vec3()
    val externalTorqueImpulse = Vec3()

    var originalBody: RigidBody? = null

    fun getVelocityInLocalPointNoDelta(relPos: Vec3, velocity: Vec3) {
        if (originalBody != null)
            velocity put (linearVelocity + externalForceImpulse + (angularVelocity + externalTorqueImpulse).cross(relPos))
        else
            velocity put 0f
    }

    fun getVelocityInLocalPointObsolete(relPos: Vec3, velocity: Vec3) {
        if (originalBody != null)
            velocity put (linearVelocity + deltaLinearVelocity + (angularVelocity + deltaAngularVelocity).cross(relPos))
        else
            velocity put 0f
    }

    fun getAngularVelocity(angVel: Vec3) {
        if (originalBody != null)
            angVel put angularVelocity + deltaAngularVelocity // TODO check order
        else
            angVel put 0f
    }

    /** Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position    */
    fun applyImpulse(linearComponent: Vec3, angularComponent: Vec3, impulseMagnitude: Float) {
        originalBody?.let {
            deltaLinearVelocity += linearComponent * impulseMagnitude * linearFactor
            deltaAngularVelocity += angularComponent * (impulseMagnitude * angularFactor)
        }
    }

    fun internalApplyPushImpulse(linearComponent: Vec3, angularComponent: Vec3, impulseMagnitude: Float) {
        originalBody?.let {
            pushVelocity += linearComponent * impulseMagnitude * linearFactor
            turnVelocity += angularComponent * (impulseMagnitude * angularFactor)
        }
    }

    ////////////////////////////////////////////////
    ///some internal methods, don't use them

    fun internalGetVelocityInLocalPointObsolete(relPos: Vec3, velocity: Vec3) {
        velocity put (linearVelocity + deltaLinearVelocity + (angularVelocity + deltaAngularVelocity).cross(relPos))
    }

    fun internalGetAngularVelocity(angVel: Vec3) = angVel.put(angularVelocity + deltaAngularVelocity)

    /** Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position    */
    fun internalApplyImpulse(linearComponent: Vec3, angularComponent: Vec3, impulseMagnitude: Float) {
        originalBody?.let {
            deltaLinearVelocity += linearComponent * impulseMagnitude * linearFactor
            deltaAngularVelocity += angularComponent * (impulseMagnitude * angularFactor)
        }
    }

    fun writebackVelocity() {
        originalBody?.let {
            linearVelocity += deltaLinearVelocity
            angularVelocity += deltaAngularVelocity
            //m_originalBody->setCompanionId(-1);
        }
    }


    fun writebackVelocityAndTransform(timeStep: Float, splitImpulseTurnErp: Float) {
        originalBody?.let {
            linearVelocity += deltaLinearVelocity
            angularVelocity += deltaAngularVelocity
            //correct the position/orientation based on push/turn recovery
            val newTransform = Transform()
            if (pushVelocity[0] != 0f || pushVelocity[1] != 0f || pushVelocity[2] != 0f || turnVelocity[0] != 0f || turnVelocity[1] != 0f || turnVelocity[2] != 0f) {
                //	btQuaternion orn = m_worldTransform.getRotation();
                TransformUtil.integrateTransform(worldTransform, pushVelocity, turnVelocity * splitImpulseTurnErp, timeStep, newTransform)
                worldTransform put newTransform
            }
            //m_worldTransform.setRotation(orn);
            //m_originalBody->setCompanionId(-1);
        }
    }
}