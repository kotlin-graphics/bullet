/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

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

//import bullet.collision.broadphaseCollision.BroadphaseProxy.CollisionFilterGroups as Cfg
import bullet.*
import bullet.collision.broadphaseCollision.BroadphaseInterface
import bullet.collision.broadphaseCollision.BroadphaseProxy
import bullet.collision.broadphaseCollision.Dispatcher
import bullet.collision.broadphaseCollision.OverlappingPairCache
import bullet.collision.collisionDispatch.*
import bullet.collision.collisionShapes.SphereShape
import bullet.collision.narrowPhaseCollision.ManifoldPoint
import bullet.collision.narrowPhaseCollision.PersistentManifold
import bullet.dynamics.constraintSolver.ConstraintSolver
import bullet.dynamics.constraintSolver.ContactSolverInfo
import bullet.dynamics.constraintSolver.SequentialImpulseConstraintSolver
import bullet.dynamics.constraintSolver.TypedConstraint
import bullet.linearMath.*
import bullet.collision.broadphaseCollision.BroadphaseProxy.CollisionFilterGroups as Cfg

/** DiscreteDynamicsWorld provides discrete rigid body simulation those classes replace the obsolete
 *  CcdPhysicsEnvironment/CcdPhysicsController  */
class DiscreteDynamicsWorld
/** this DiscreteDynamicsWorld constructor gets created objects from the user, and will not delete those */
constructor(dispatcher: Dispatcher?, pairCache: BroadphaseInterface, constraintSolver: ConstraintSolver?,
            collisionConfiguration: CollisionConfiguration) : DynamicsWorld(dispatcher, pairCache, collisionConfiguration) {

    var ownsConstraintSolver = false
    override var constraintSolver = constraintSolver
            ?: SequentialImpulseConstraintSolver().also { ownsConstraintSolver = true }
        set(value) {
            ownsConstraintSolver = false
            field = value
            solverIslandCallback.solver = value
        }

    val sortedConstraints = ArrayList<TypedConstraint>()
    var solverIslandCallback = InplaceSolverIslandCallback(this.constraintSolver, dispatcher!!)

    var islandManager = SimulationIslandManager()
    var ownsIslandManager = true

    val constraints = ArrayList<TypedConstraint>()

    val nonStaticRigidBodies = ArrayList<RigidBody>()

    override var gravity = Vec3(0, -10, 0)
        set(value) {
            field put value
            nonStaticRigidBodies.filter { it.isActive && it.rigidbodyFlags hasnt RigidBodyFlags.DISABLE_WORLD_GRAVITY }
                    .forEach { it.gravity = gravity }
        }

    //for variable timesteps
    var localTime = 0f
    var fixedTimeStep = 0f
    //for variable timesteps

    var synchronizeAllMotionStates = false
    var applySpeculativeContactRestitution = false

    val actions = ArrayList<ActionInterface>()

    var profileTimings = 0

    var latencyMotionStateInterpolation = true

    val predictiveManifolds = ArrayList<PersistentManifold>()
    /** used to synchronize threads creating predictive contacts    */
    val predictiveManifoldsMutex = SpinMutex()

    fun predictUnconstraintMotion(timeStep: Float) {
        BT_PROFILE("predictUnconstraintMotion")
        nonStaticRigidBodies.forEach {
            if (!it.isStaticOrKinematicObject) {
                // don't integrate/update velocities here, it happens in the constraint solver
                it.applyDamping(timeStep)
                it.predictIntegratedTransform(timeStep, it.getInterpolationWorldTransform())
            }
        }
    }

    /** can be called in parallel */
    fun integrateTransformsInternal(bodies: ArrayList<RigidBody>, numBodies: Int, timeStep: Float) {

        val predictedTrans = Transform()
        for (i in 0 until numBodies) {
            val body = bodies[i].apply { hitFraction = 1f }
            if (body.isActive && !body.isStaticOrKinematicObject) {

                body.predictIntegratedTransform(timeStep, predictedTrans)
                val squareMotion = (predictedTrans.origin - body.getWorldTransform().origin).length2()

                if (dispatchInfo.useContinuous && body.ccdSquareMotionThreshold != 0f && body.ccdSquareMotionThreshold < squareMotion) {
                    BT_PROFILE("CCD motion clamping")
                    if (body.collisionShape!!.isConvex) {
                        gNumClampedCcdMotions++
                        if (USE_STATIC_ONLY) {
                            TODO()
//                            class StaticOnlyCallback(btCollisionObject * me, const btVector3 & fromA, const btVector3 & toA, btOverlappingPairCache * pairCache, btDispatcher * dispatcher) : ClosestNotMeConvexResultCallback {
//
//                                StaticOnlyCallback() :
//                                btClosestNotMeConvexResultCallback(me, fromA, toA, pairCache, dispatcher)
//                                {
//                                }
//
//                                virtual bool needsCollision(btBroadphaseProxy * proxy0) const
//                                        {
//                                            btCollisionObject * otherObj = (btCollisionObject *) proxy0->m_clientObject
//                                            if (!otherObj->isStaticOrKinematicObject())
//                                            return false
//                                            return btClosestNotMeConvexResultCallback::needsCollision(proxy0)
//                                        }
//                            }
//
//                            StaticOnlyCallback sweepResults (body, body->getWorldTransform().getOrigin(), predictedTrans.getOrigin(), getBroadphase()->getOverlappingPairCache(), getDispatcher())
                        } else Unit
                        val sweepResults = ClosestNotMeConvexResultCallback(body, body.getWorldTransform().origin,
                                predictedTrans.origin, broadphase.overlappingPairCache, dispatcher!!)

                        //btConvexShape* convexShape = static_cast<btConvexShape*>(body->getCollisionShape());
                        val tmpSphere = SphereShape(body.ccdSweptSphereRadius)//btConvexShape* convexShape = static_cast<btConvexShape*>(body->getCollisionShape());
                        sweepResults.allowedPenetration = dispatchInfo.allowedCcdPenetration

                        sweepResults.collisionFilterGroup = body.broadphaseHandle!!.collisionFilterGroup
                        sweepResults.collisionFilterMask = body.broadphaseHandle!!.collisionFilterMask
                        val modifiedPredictedTrans = Transform(predictedTrans)
                        modifiedPredictedTrans.basis = body.getWorldTransform().basis

                        convexSweepTest(tmpSphere, body.getWorldTransform(), modifiedPredictedTrans, sweepResults)
                        if (sweepResults.hasHit && sweepResults.closestHitFraction < 1f) {
                            //printf("clamped integration to hit fraction = %f\n",fraction);
                            with(body) {
                                hitFraction = sweepResults.closestHitFraction
                                predictIntegratedTransform(timeStep * hitFraction, predictedTrans)
                                hitFraction = 0f
                                proceedToTransform(predictedTrans)
                            }
                            /*  Don't apply the collision response right now, it will happen next frame
                                if you really need to, you can uncomment next 3 lines. Note that is uses zero restitution.
                             */
                            //btScalar appliedImpulse = 0.f;
                            //btScalar depth = 0.f;
                            //appliedImpulse = resolveSingleCollision(body,(btCollisionObject*)sweepResults.m_hitCollisionObject,sweepResults.m_hitPointWorld,sweepResults.m_hitNormalWorld,getSolverInfo(), depth);
                            continue
                        }
                    }
                }
                body.proceedToTransform(predictedTrans)
            }
        }
    }

    fun integrateTransforms(timeStep: Float) {
        BT_PROFILE("integrateTransforms")
        if (nonStaticRigidBodies.isNotEmpty())
            integrateTransformsInternal(nonStaticRigidBodies, nonStaticRigidBodies.size, timeStep)
        ///this should probably be switched on by default, but it is not well tested yet
        if (applySpeculativeContactRestitution) {
            BT_PROFILE("apply speculative contact restitution")
            for (i in 0 until predictiveManifolds.size) {
                val manifold = predictiveManifolds[i]
                val body0 = RigidBody.upcast(manifold.body0!!)!!
                val body1 = RigidBody.upcast(manifold.body1!!)!!

                for (p in 0 until manifold.numContacts) {
                    val pt = manifold.getContactPoint(p)
                    val combinedRestitution = gCalculateCombinedRestitutionCallback(body0, body1)
                    if (combinedRestitution > 0 && pt.appliedImpulse != 0f)
                    //if (pt.getDistance()>0 && combinedRestitution>0 && pt.m_appliedImpulse != 0.f)
                    {
                        val imp = -pt.normalWorldOnB * pt.appliedImpulse * combinedRestitution

                        val pos1 = pt.positionWorldOnA
                        val pos2 = pt.positionWorldOnB

                        val relPos0 = pos1 - body0.getWorldTransform().origin
                        val relPos1 = pos2 - body1.getWorldTransform().origin

                        body0.applyImpulse(imp, relPos0)
                        body1.applyImpulse(imp, relPos0)
                    }
                }
            }
        }
    }

    fun calculateSimulationIslands() {
        BT_PROFILE("calculateSimulationIslands")
        islandManager.updateActivationState(this, dispatcher!!)

        //merge islands based on speculative contact manifolds too
        predictiveManifolds.forEach {
            val colObj0 = it.body0
            val colObj1 = it.body1

            if (colObj0 != null && !colObj0.isStaticOrKinematicObject && colObj1 != null && !colObj1.isStaticOrKinematicObject)
                islandManager.unionFind.unite(colObj0.islandTag, colObj1.islandTag)
        }
        constraints.filter { it.isEnabled }.forEach {
            val colObj0 = it.rbA
            val colObj1 = it.rbB
            if (colObj0 != null && !colObj0.isStaticOrKinematicObject && colObj1 != null && !colObj1.isStaticOrKinematicObject)
                islandManager.unionFind.unite(colObj0.islandTag, colObj1.islandTag)
        }
        //Store the island id in each body
        islandManager.storeIslandActivationState(this)
    }

    fun solveConstraints(solverInfo: ContactSolverInfo) {
        BT_PROFILE("solveConstraints")
        sortedConstraints resize constraints.size
        for (i in 0 until numConstraints)
            sortedConstraints[i] = constraints[i]
//	btAssert(0);
        sortedConstraints.sortWith(SortConstraintOnIslandPredicate)
        val constraintsPtr = sortedConstraints.takeIf { numConstraints != 0 }

        solverIslandCallback.setup(solverInfo, constraintsPtr, sortedConstraints.size, debugDrawer)
        constraintSolver.prepareSolve(numCollisionObjects, dispatcher!!.numManifolds)
        // solve all the constraints for this island
        islandManager.buildAndProcessIslands(dispatcher!!, this, solverIslandCallback)
        solverIslandCallback.processConstraints()
        constraintSolver.allSolved(solverInfo, debugDrawer)
    }

    fun updateActivationState(timeStep: Float) {
        BT_PROFILE("updateActivationState")
        nonStaticRigidBodies.forEach {
            //            if (body) { TODO check nullability
            it.updateDeactivation(timeStep)
            if (it.wantsSleeping) {
                if (it.isStaticOrKinematicObject)
                    it.activationState = ISLAND_SLEEPING
                else {
                    if (it.activationState == ACTIVE_TAG)
                        it.activationState = WANTS_DEACTIVATION
                    if (it.activationState == ISLAND_SLEEPING) {
                        it.setAngularVelocity(Vec3())
                        it.setLinearVelocity(Vec3())
                    }
                }
            } else
                if (it.activationState != DISABLE_DEACTIVATION)
                    it.activationState = ACTIVE_TAG
//                }
        }
    }

    fun updateActions(timeStep: Float) {
        BT_PROFILE("updateActions")
        actions.forEach { it.updateAction(this, timeStep) }
    }

    fun startProfiling(timeStep: Float) {
        if (!BT_NO_PROFILE)
            TODO()//CProfileManager::Reset();

    }

    fun internalSingleStepSimulation(timeStep: Float) {
        BT_PROFILE("internalSingleStepSimulation")
        internalPreTickCallback?.invoke(this, timeStep)
        // apply gravity, predict motion
        predictUnconstraintMotion(timeStep)

        dispatchInfo.timeStep = timeStep
        dispatchInfo.stepCount = 0
        dispatchInfo.debugDraw = debugDrawer

        createPredictiveContacts(timeStep)
        // perform collision detection
        performDiscreteCollisionDetection()

        calculateSimulationIslands()

        solverInfo.timeStep = timeStep
        // solve contact and other joint constraints
        solveConstraints(solverInfo)

        ///CallbackTriggers();
        // integrate transforms
        integrateTransforms(timeStep)
        // update vehicle simulation
        updateActions(timeStep)

        updateActivationState(timeStep)

        internalTickCallback?.invoke(this, timeStep)
    }

    fun releasePredictiveContacts() {
        BT_PROFILE("release predictive contact manifolds")
        predictiveManifolds.forEach(dispatcher!!::releaseManifold)
        predictiveManifolds.clear()
    }

    /** can be called in parallel */
    fun createPredictiveContactsInternal(bodies: ArrayList<RigidBody>, numBodies: Int, timeStep: Float) {
        val predictedTrans = Transform()
        for (i in 0 until numBodies) {
            val body = bodies[i].apply { hitFraction = 1f }
            if (body.isActive && !body.isStaticOrKinematicObject) {
                body.predictIntegratedTransform(timeStep, predictedTrans)

                val squareMotion = (predictedTrans.origin - body.getWorldTransform().origin).length2()

                if (dispatchInfo.useContinuous && body.ccdSquareMotionThreshold != 0f && body.ccdSquareMotionThreshold < squareMotion) {
                    BT_PROFILE("predictive convexSweepTest")
                    if (body.collisionShape!!.isConvex) {
                        gNumClampedCcdMotions++
//                        #ifdef PREDICTIVE_CONTACT_USE_STATIC_ONLY TODO
//                        class StaticOnlyCallback : public btClosestNotMeConvexResultCallback
//                        {
//                            public:
//
//                            StaticOnlyCallback(btCollisionObject * me, const btVector3 & fromA, const btVector3 & toA, btOverlappingPairCache * pairCache, btDispatcher * dispatcher) :
//                            btClosestNotMeConvexResultCallback(me, fromA, toA, pairCache, dispatcher)
//                            {
//                            }
//
//                            virtual bool needsCollision(btBroadphaseProxy * proxy0) const
//                                    {
//                                        btCollisionObject * otherObj = (btCollisionObject *) proxy0->m_clientObject
//                                        if (!otherObj->isStaticOrKinematicObject())
//                                        return false
//                                        return btClosestNotMeConvexResultCallback::needsCollision(proxy0)
//                                    }
//                        }
//
//                        StaticOnlyCallback sweepResults (body, body->getWorldTransform().getOrigin(), predictedTrans.getOrigin(), getBroadphase()->getOverlappingPairCache(), getDispatcher())
//                        #else
                        val sweepResults = ClosestNotMeConvexResultCallback(body, body.getWorldTransform().origin,
                                predictedTrans.origin, broadphase.overlappingPairCache, dispatcher!!)
//                        #endif
                        val tmpSphere = SphereShape(body.ccdSweptSphereRadius)//btConvexShape* convexShape = static_cast<btConvexShape*>(body->getCollisionShape());
                        sweepResults.allowedPenetration = dispatchInfo.allowedCcdPenetration

                        sweepResults.collisionFilterGroup = body.broadphaseHandle!!.collisionFilterGroup
                        sweepResults.collisionFilterMask = body.broadphaseHandle!!.collisionFilterMask
                        val modifiedPredictedTrans = Transform(predictedTrans)
                        modifiedPredictedTrans.basis = body.getWorldTransform().basis

                        convexSweepTest(tmpSphere, body.getWorldTransform(), modifiedPredictedTrans, sweepResults)
                        if (sweepResults.hasHit && sweepResults.closestHitFraction < 1f) {

                            val distVec = (predictedTrans.origin - body.getWorldTransform().origin) * sweepResults.closestHitFraction
                            val distance = distVec dot -sweepResults.hitNormalWorld

                            val manifold = dispatcher!!.getNewManifold(body, sweepResults.hitCollisionObject!!)
//                            btMutexLock(& m_predictiveManifoldsMutex ) TODO
                            predictiveManifolds.add(manifold)
//                            btMutexUnlock(& m_predictiveManifoldsMutex )

                            val worldPointB = body.getWorldTransform().origin + distVec
                            val localPointB = sweepResults.hitCollisionObject!!.getWorldTransform().inverse() * worldPointB

                            val newPoint = ManifoldPoint(Vec3(), localPointB, sweepResults.hitNormalWorld, distance)

                            val index = manifold.addManifoldPoint(newPoint, isPredictive = true)
                            manifold.getContactPoint(index).apply {
                                combinedRestitution = 0f
                                combinedFriction = gCalculateCombinedFrictionCallback(body, sweepResults.hitCollisionObject!!)
                                positionWorldOnA put body.getWorldTransform().origin
                                positionWorldOnB put worldPointB
                            }
                        }
                    }
                }
            }
        }
    }

    fun createPredictiveContacts(timeStep: Float) {
        BT_PROFILE("createPredictiveContacts")
        releasePredictiveContacts()
        if (nonStaticRigidBodies.isNotEmpty())
            createPredictiveContactsInternal(nonStaticRigidBodies, nonStaticRigidBodies.size, timeStep)
    }

    fun saveKinematicState(timeStep: Float) {
        /*  would like to iterate over nonStaticRigidBodies, but unfortunately old API allows to switch status _after_
            adding kinematic objects to the world
            fix it for Bullet 3.x release */
        collisionObjects.forEach {
            RigidBody.upcast(it)?.let {
                if (it.activationState != ISLAND_SLEEPING && it.isKinematicObject)
                    it.saveKinematicState(timeStep) //to calculate velocities next frame
            }
        }
    }

    /** if maxSubSteps > 0, it will interpolate motion between fixedTimeStep's */
    fun stepSimulation(timeStep: Float) = stepSimulation(timeStep, maxSubSteps = 1, fixedTimeStep = 1f / 60f)

    override fun stepSimulation(timeStep: Float, maxSubSteps: Int, fixedTimeStep: Float): Int {

        startProfiling(timeStep)

        var numSimulationSubSteps = 0
        var fixedTimeStep = fixedTimeStep
        var maxSubSteps = maxSubSteps

        if (maxSubSteps != 0) {
            //fixed timestep with interpolation
            this.fixedTimeStep = fixedTimeStep
            localTime += timeStep
            if (localTime >= fixedTimeStep) {
                numSimulationSubSteps = (localTime / fixedTimeStep).i
                localTime -= numSimulationSubSteps * fixedTimeStep
            }
        } else {
            //variable timestep
            fixedTimeStep = timeStep
            localTime = if (latencyMotionStateInterpolation) 0f else timeStep
            fixedTimeStep = 0f
            if (timeStep.fuzzyZero) {
                numSimulationSubSteps = 0
                maxSubSteps = 0
            } else {
                numSimulationSubSteps = 1
                maxSubSteps = 1
            }
        }
        //process some debugging flags
        debugDrawer?.let {
            TODO()
//            gDisableDeactivation = (it.getDebugMode() & btIDebugDraw::DBG_NoDeactivation) != 0
        }
        if (numSimulationSubSteps != 0) {
            //clamp the number of substeps, to prevent simulation grinding spiralling down to a halt
            val clampedSimulationSteps = if (numSimulationSubSteps > maxSubSteps) maxSubSteps else numSimulationSubSteps
            saveKinematicState(fixedTimeStep * clampedSimulationSteps)
            applyGravity()
            for (i in 0 until clampedSimulationSteps) {
                internalSingleStepSimulation(fixedTimeStep)
                synchronizeMotionStates()
            }
        } else synchronizeMotionStates()

        clearForces()

        if (!BT_NO_PROFILE) TODO() // CProfileManager::Increment_Frame_Counter()

        return numSimulationSubSteps
    }

    override fun synchronizeMotionStates() {
        BT_PROFILE("synchronizeMotionStates")
        if (synchronizeAllMotionStates)
        //iterate  over all collision objects
            for (it in collisionObjects)
                RigidBody.upcast(it)?.let { synchronizeSingleMotionState(it) }
        //iterate over all active rigid bodies
        else nonStaticRigidBodies.filter { it.isActive }.forEach(::synchronizeSingleMotionState)
    }

    /** this can be useful to synchronize a single rigid body -> graphics object */
    fun synchronizeSingleMotionState(body: RigidBody) {
//        btAssert(body);
        if (body.motionState != null && !body.isStaticOrKinematicObject) {
            /*  we need to call the update at least once, even for sleeping objects otherwise the 'graphics' transform
                never updates properly
                todo: add 'dirty' flag */
            //if (body->getActivationState() != ISLAND_SLEEPING)
            val interpolatedTransform = Transform()
            val timeStep = if (latencyMotionStateInterpolation && fixedTimeStep != 0f) localTime - fixedTimeStep else localTime * body.hitFraction
            TransformUtil.integrateTransform(body.getInterpolationWorldTransform(), body.getInterpolationLinearVelocity(),
                    body.getInterpolationAngularVelocity(), timeStep, interpolatedTransform)
            body.motionState!!.setWorldTransform(interpolatedTransform)
        }
    }

    fun addConstraint(constraint: TypedConstraint) = addConstraint(constraint, disableCollisionsBetweenLinkedBodies = false)
    override fun addConstraint(constraint: TypedConstraint, disableCollisionsBetweenLinkedBodies: Boolean) {
        constraints.add(constraint)
        // Make sure the two bodies of a type constraint are different (possibly add this to the btTypedConstraint constructor?)
        assert(constraint.rbA !== constraint.rbB)
        if (disableCollisionsBetweenLinkedBodies) {
            constraint.rbA!!.addConstraintRef(constraint)
            constraint.rbB!!.addConstraintRef(constraint)
        }
    }

    override fun removeConstraint(constraint: TypedConstraint) {
        constraints.remove(constraint)
        constraint.rbA!!.removeConstraintRef(constraint)
        constraint.rbB!!.removeConstraintRef(constraint)
    }

    override fun addAction(action: ActionInterface) {
        actions += action
    }

    override fun removeAction(action: ActionInterface) {
        actions -= action
    }

    override fun addRigidBody(body: RigidBody) {

        if (!body.isStaticOrKinematicObject && body.rigidbodyFlags hasnt RigidBodyFlags.DISABLE_WORLD_GRAVITY)
            body.gravity = gravity

        if (body.collisionShape != null) {
            if (!body.isStaticObject)
                nonStaticRigidBodies += body
            else
                body.activationState = ISLAND_SLEEPING

            val isDynamic = !body.isStaticObject && !body.isKinematicObject
            val collisionFilterGroup = if (isDynamic) Cfg.DefaultFilter else Cfg.StaticFilter
            val collisionFilterMask =
                    if (isDynamic) Cfg.AllFilter.i
                    else Cfg.AllFilter.i xor Cfg.StaticFilter.i

            addCollisionObject(body, collisionFilterGroup.i, collisionFilterMask)
        }
    }

    override fun addRigidBody(body: RigidBody, group: Int, mask: Int) {
        if (!body.isStaticOrKinematicObject && body.rigidbodyFlags hasnt RigidBodyFlags.DISABLE_WORLD_GRAVITY)
            body.gravity = gravity
        body.collisionShape?.let {
            if (!body.isStaticObject)
                nonStaticRigidBodies.add(body)
            else
                body.activationState = ISLAND_SLEEPING
            addCollisionObject(body, group, mask)
        }
    }

    override fun removeRigidBody(body: RigidBody) {
        nonStaticRigidBodies.remove(body)
        super.removeCollisionObject(body)
    }

    /** removeCollisionObject will first check if it is a rigid body, if so call removeRigidBody otherwise call
     *  CollisionWorld::removeCollisionObject */
    override fun removeCollisionObject(collisionObject: CollisionObject) {
        val body = RigidBody.upcast(collisionObject)
        if (body != null)
            removeRigidBody(body)
        else
            super.removeCollisionObject(collisionObject)
    }

    fun debugDrawConstraint(constraint: TypedConstraint) {
        TODO()
        /*
        bool drawFrames = (getDebugDrawer()->getDebugMode() & btIDebugDraw::DBG_DrawConstraints) != 0;
	bool drawLimits = (getDebugDrawer()->getDebugMode() & btIDebugDraw::DBG_DrawConstraintLimits) != 0;
	btScalar dbgDrawSize = constraint->getDbgDrawSize();
	if(dbgDrawSize <= btScalar(0.f))
	{
		return;
	}

	switch(constraint->getConstraintType())
	{
		case POINT2POINT_CONSTRAINT_TYPE:
			{
				btPoint2PointConstraint* p2pC = (btPoint2PointConstraint*)constraint;
				btTransform tr;
				tr.setIdentity();
				btVector3 pivot = p2pC->getPivotInA();
				pivot = p2pC->getRigidBodyA().getCenterOfMassTransform() * pivot;
				tr.setOrigin(pivot);
				getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				// that ideally should draw the same frame
				pivot = p2pC->getPivotInB();
				pivot = p2pC->getRigidBodyB().getCenterOfMassTransform() * pivot;
				tr.setOrigin(pivot);
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			}
			break;
		case HINGE_CONSTRAINT_TYPE:
			{
				btHingeConstraint* pHinge = (btHingeConstraint*)constraint;
				btTransform tr = pHinge->getRigidBodyA().getCenterOfMassTransform() * pHinge->getAFrame();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = pHinge->getRigidBodyB().getCenterOfMassTransform() * pHinge->getBFrame();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				btScalar minAng = pHinge->getLowerLimit();
				btScalar maxAng = pHinge->getUpperLimit();
				if(minAng == maxAng)
				{
					break;
				}
				bool drawSect = true;
				if(!pHinge->hasLimit())
				{
					minAng = btScalar(0.f);
					maxAng = SIMD_2_PI;
					drawSect = false;
				}
				if(drawLimits)
				{
					btVector3& center = tr.getOrigin();
					btVector3 normal = tr.getBasis().getColumn(2);
					btVector3 axis = tr.getBasis().getColumn(0);
					getDebugDrawer()->drawArc(center, normal, axis, dbgDrawSize, dbgDrawSize, minAng, maxAng, btVector3(0,0,0), drawSect);
				}
			}
			break;
		case CONETWIST_CONSTRAINT_TYPE:
			{
				btConeTwistConstraint* pCT = (btConeTwistConstraint*)constraint;
				btTransform tr = pCT->getRigidBodyA().getCenterOfMassTransform() * pCT->getAFrame();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = pCT->getRigidBodyB().getCenterOfMassTransform() * pCT->getBFrame();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				if(drawLimits)
				{
					//const btScalar length = btScalar(5);
					const btScalar length = dbgDrawSize;
					static int nSegments = 8*4;
					btScalar fAngleInRadians = btScalar(2.*3.1415926) * (btScalar)(nSegments-1)/btScalar(nSegments);
					btVector3 pPrev = pCT->GetPointForAngle(fAngleInRadians, length);
					pPrev = tr * pPrev;
					for (int i=0; i<nSegments; i++)
					{
						fAngleInRadians = btScalar(2.*3.1415926) * (btScalar)i/btScalar(nSegments);
						btVector3 pCur = pCT->GetPointForAngle(fAngleInRadians, length);
						pCur = tr * pCur;
						getDebugDrawer()->drawLine(pPrev, pCur, btVector3(0,0,0));

						if (i%(nSegments/8) == 0)
							getDebugDrawer()->drawLine(tr.getOrigin(), pCur, btVector3(0,0,0));

						pPrev = pCur;
					}
					btScalar tws = pCT->getTwistSpan();
					btScalar twa = pCT->getTwistAngle();
					bool useFrameB = (pCT->getRigidBodyB().getInvMass() > btScalar(0.f));
					if(useFrameB)
					{
						tr = pCT->getRigidBodyB().getCenterOfMassTransform() * pCT->getBFrame();
					}
					else
					{
						tr = pCT->getRigidBodyA().getCenterOfMassTransform() * pCT->getAFrame();
					}
					btVector3 pivot = tr.getOrigin();
					btVector3 normal = tr.getBasis().getColumn(0);
					btVector3 axis1 = tr.getBasis().getColumn(1);
					getDebugDrawer()->drawArc(pivot, normal, axis1, dbgDrawSize, dbgDrawSize, -twa-tws, -twa+tws, btVector3(0,0,0), true);

				}
			}
			break;
		case D6_SPRING_CONSTRAINT_TYPE:
		case D6_CONSTRAINT_TYPE:
			{
				btGeneric6DofConstraint* p6DOF = (btGeneric6DofConstraint*)constraint;
				btTransform tr = p6DOF->getCalculatedTransformA();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = p6DOF->getCalculatedTransformB();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				if(drawLimits)
				{
					tr = p6DOF->getCalculatedTransformA();
					const btVector3& center = p6DOF->getCalculatedTransformB().getOrigin();
					btVector3 up = tr.getBasis().getColumn(2);
					btVector3 axis = tr.getBasis().getColumn(0);
					btScalar minTh = p6DOF->getRotationalLimitMotor(1)->m_loLimit;
					btScalar maxTh = p6DOF->getRotationalLimitMotor(1)->m_hiLimit;
					btScalar minPs = p6DOF->getRotationalLimitMotor(2)->m_loLimit;
					btScalar maxPs = p6DOF->getRotationalLimitMotor(2)->m_hiLimit;
					getDebugDrawer()->drawSpherePatch(center, up, axis, dbgDrawSize * btScalar(.9f), minTh, maxTh, minPs, maxPs, btVector3(0,0,0));
					axis = tr.getBasis().getColumn(1);
					btScalar ay = p6DOF->getAngle(1);
					btScalar az = p6DOF->getAngle(2);
					btScalar cy = btCos(ay);
					btScalar sy = btSin(ay);
					btScalar cz = btCos(az);
					btScalar sz = btSin(az);
					btVector3 ref;
					ref[0] = cy*cz*axis[0] + cy*sz*axis[1] - sy*axis[2];
					ref[1] = -sz*axis[0] + cz*axis[1];
					ref[2] = cz*sy*axis[0] + sz*sy*axis[1] + cy*axis[2];
					tr = p6DOF->getCalculatedTransformB();
					btVector3 normal = -tr.getBasis().getColumn(0);
					btScalar minFi = p6DOF->getRotationalLimitMotor(0)->m_loLimit;
					btScalar maxFi = p6DOF->getRotationalLimitMotor(0)->m_hiLimit;
					if(minFi > maxFi)
					{
						getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, -SIMD_PI, SIMD_PI, btVector3(0,0,0), false);
					}
					else if(minFi < maxFi)
					{
						getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, minFi, maxFi, btVector3(0,0,0), true);
					}
					tr = p6DOF->getCalculatedTransformA();
					btVector3 bbMin = p6DOF->getTranslationalLimitMotor()->m_lowerLimit;
					btVector3 bbMax = p6DOF->getTranslationalLimitMotor()->m_upperLimit;
					getDebugDrawer()->drawBox(bbMin, bbMax, tr, btVector3(0,0,0));
				}
			}
			break;
		///note: the code for D6_SPRING_2_CONSTRAINT_TYPE is identical to D6_CONSTRAINT_TYPE, the D6_CONSTRAINT_TYPE+D6_SPRING_CONSTRAINT_TYPE will likely become obsolete/deprecated at some stage
		case D6_SPRING_2_CONSTRAINT_TYPE:
		{
			{
				btGeneric6DofSpring2Constraint* p6DOF = (btGeneric6DofSpring2Constraint*)constraint;
				btTransform tr = p6DOF->getCalculatedTransformA();
				if (drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = p6DOF->getCalculatedTransformB();
				if (drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				if (drawLimits)
				{
					tr = p6DOF->getCalculatedTransformA();
					const btVector3& center = p6DOF->getCalculatedTransformB().getOrigin();
					btVector3 up = tr.getBasis().getColumn(2);
					btVector3 axis = tr.getBasis().getColumn(0);
					btScalar minTh = p6DOF->getRotationalLimitMotor(1)->m_loLimit;
					btScalar maxTh = p6DOF->getRotationalLimitMotor(1)->m_hiLimit;
					btScalar minPs = p6DOF->getRotationalLimitMotor(2)->m_loLimit;
					btScalar maxPs = p6DOF->getRotationalLimitMotor(2)->m_hiLimit;
					getDebugDrawer()->drawSpherePatch(center, up, axis, dbgDrawSize * btScalar(.9f), minTh, maxTh, minPs, maxPs, btVector3(0, 0, 0));
					axis = tr.getBasis().getColumn(1);
					btScalar ay = p6DOF->getAngle(1);
					btScalar az = p6DOF->getAngle(2);
					btScalar cy = btCos(ay);
					btScalar sy = btSin(ay);
					btScalar cz = btCos(az);
					btScalar sz = btSin(az);
					btVector3 ref;
					ref[0] = cy*cz*axis[0] + cy*sz*axis[1] - sy*axis[2];
					ref[1] = -sz*axis[0] + cz*axis[1];
					ref[2] = cz*sy*axis[0] + sz*sy*axis[1] + cy*axis[2];
					tr = p6DOF->getCalculatedTransformB();
					btVector3 normal = -tr.getBasis().getColumn(0);
					btScalar minFi = p6DOF->getRotationalLimitMotor(0)->m_loLimit;
					btScalar maxFi = p6DOF->getRotationalLimitMotor(0)->m_hiLimit;
					if (minFi > maxFi)
					{
						getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, -SIMD_PI, SIMD_PI, btVector3(0, 0, 0), false);
					}
					else if (minFi < maxFi)
					{
						getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, minFi, maxFi, btVector3(0, 0, 0), true);
					}
					tr = p6DOF->getCalculatedTransformA();
					btVector3 bbMin = p6DOF->getTranslationalLimitMotor()->m_lowerLimit;
					btVector3 bbMax = p6DOF->getTranslationalLimitMotor()->m_upperLimit;
					getDebugDrawer()->drawBox(bbMin, bbMax, tr, btVector3(0, 0, 0));
				}
			}
			break;
		}
		case SLIDER_CONSTRAINT_TYPE:
			{
				btSliderConstraint* pSlider = (btSliderConstraint*)constraint;
				btTransform tr = pSlider->getCalculatedTransformA();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = pSlider->getCalculatedTransformB();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				if(drawLimits)
				{
					btTransform tr = pSlider->getUseLinearReferenceFrameA() ? pSlider->getCalculatedTransformA() : pSlider->getCalculatedTransformB();
					btVector3 li_min = tr * btVector3(pSlider->getLowerLinLimit(), 0.f, 0.f);
					btVector3 li_max = tr * btVector3(pSlider->getUpperLinLimit(), 0.f, 0.f);
					getDebugDrawer()->drawLine(li_min, li_max, btVector3(0, 0, 0));
					btVector3 normal = tr.getBasis().getColumn(0);
					btVector3 axis = tr.getBasis().getColumn(1);
					btScalar a_min = pSlider->getLowerAngLimit();
					btScalar a_max = pSlider->getUpperAngLimit();
					const btVector3& center = pSlider->getCalculatedTransformB().getOrigin();
					getDebugDrawer()->drawArc(center, normal, axis, dbgDrawSize, dbgDrawSize, a_min, a_max, btVector3(0,0,0), true);
				}
			}
			break;
		default :
			break;
	}
	return;
         */
    }

    override fun debugDrawWorld() {
        TODO()
        /*
        BT_PROFILE("debugDrawWorld")

	btCollisionWorld::debugDrawWorld();

	bool drawConstraints = false;
	if (getDebugDrawer())
	{
		int mode = getDebugDrawer()->getDebugMode();
		if(mode  & (btIDebugDraw::DBG_DrawConstraints | btIDebugDraw::DBG_DrawConstraintLimits))
		{
			drawConstraints = true;
		}
	}
	if(drawConstraints)
	{
		for(int i = getNumConstraints()-1; i>=0 ;i--)
		{
			btTypedConstraint* constraint = getConstraint(i);
			debugDrawConstraint(constraint);
		}
	}



    if (getDebugDrawer() && (getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawAabb | btIDebugDraw::DBG_DrawNormals)))
	{
		int i;

		if (getDebugDrawer() && getDebugDrawer()->getDebugMode())
		{
			for (i=0;i<m_actions.size();i++)
			{
				m_actions[i]->debugDraw(m_debugDrawer);
			}
		}
	}
    if (getDebugDrawer())
        getDebugDrawer()->flushLines();
         */
    }

    override val numConstraints get() = constraints.size

    override fun getConstraint(index: Int) = constraints[index]

    override val worldType get() = DynamicsWorldType.DISCRETE_DYNAMICS_WORLD

    /** the forces on each rigidbody is accumulating together with gravity. clear this after each timestep. */
    override fun clearForces() {
        ///@todo: iterate over awake simulation islands!
        //need to check if next line is ok
        //it might break backward compatibility (people applying forces on sleeping objects get never cleared and accumulate on wake-up
        nonStaticRigidBodies.forEach { it.clearForces() }
    }

    /** apply gravity, call this once per timestep */
    fun applyGravity() {
        ///@todo: iterate over awake simulation islands!
        nonStaticRigidBodies.filter { it.isActive }.forEach { it.applyGravity() }
    }

    open var numTask = 0
}

val TypedConstraint.islandId get() = if (rbA!!.islandTag >= 0) rbA!!.islandTag else rbB!!.islandTag

object SortConstraintOnIslandPredicate : Comparator<TypedConstraint> {
    override fun compare(o1: TypedConstraint, o2: TypedConstraint) = o1.islandId.compareTo(o2.islandId)
}

class InplaceSolverIslandCallback(var solver: ConstraintSolver, val dispatcher: Dispatcher) : SimulationIslandManager.IslandCallback {

    var solverInfo: ContactSolverInfo? = null
    val sortedConstraints = ArrayList<TypedConstraint>()
    var numConstraints = 0
    var debugDrawer: DebugDraw? = null

    val bodies = ArrayList<CollisionObject>()
    val manifolds = ArrayList<PersistentManifold>()
    val constraints = ArrayList<TypedConstraint>()

    fun setup(solverInfo: ContactSolverInfo, sortedConstraints: ArrayList<TypedConstraint>?, numConstraints: Int, debugDrawer: DebugDraw?) {
        this.solverInfo = solverInfo
        this.sortedConstraints.clear()
        sortedConstraints?.let { this.sortedConstraints += it }
        this.numConstraints = numConstraints
        this.debugDrawer = debugDrawer
    }

    override fun processIsland(bodies: ArrayList<CollisionObject>, numBodies: Int, manifolds: ArrayList<PersistentManifold>,
                               manifoldsPtr: Int, numManifolds: Int, islandId: Int) {
        if (islandId < 0)
        ///we don't split islands, so all constraints/contact manifolds/bodies are passed into the solver regardless the island id
            solver.solveGroup(bodies, numBodies, manifolds, manifoldsPtr, numManifolds, sortedConstraints, 0, numConstraints,
                    solverInfo!!, debugDrawer, dispatcher)
        else {
            // also add all non-contact constraints/joints for this island
            var startConstraint = 0
            var numCurConstraints = 0
            //find the first constraint for this island
            var i = 0
            while (i < numConstraints) {
                if (sortedConstraints[i].islandId == islandId) {
                    startConstraint = i
                    break
                }
                ++i
            }
            //count the number of constraints in this island
            while (i < numConstraints) {
                if (sortedConstraints[i].islandId == islandId)
                    numCurConstraints++
                i++
            }
            if (solverInfo!!.minimumSolverBatchSize <= 1)
                solver.solveGroup(bodies, numBodies, manifolds, manifoldsPtr, numManifolds, sortedConstraints, startConstraint,
                        numCurConstraints, solverInfo!!, debugDrawer, dispatcher)
            else {
                for (j in 0 until numBodies) this.bodies += bodies[j]
                for (j in 0 until numManifolds) this.manifolds += manifolds[manifoldsPtr + j]
                for (j in 0 until numCurConstraints) this.constraints += sortedConstraints[startConstraint + j]
                if ((constraints.size + manifolds.size) > solverInfo!!.minimumSolverBatchSize)
                    processConstraints()
                else println("deferred")
            }
        }
    }

    fun processConstraints() {
        solver.solveGroup(bodies, bodies.size, manifolds, 0, manifolds.size, constraints, 0, constraints.size,
                solverInfo!!, debugDrawer, dispatcher)
        bodies.clear()
        manifolds.clear()
        constraints.clear()
    }
}

class ClosestNotMeConvexResultCallback(val me: CollisionObject, fromA: Vec3, toA: Vec3, val pairCache: OverlappingPairCache,
                                       val dispatcher: Dispatcher) : CollisionWorld.ClosestConvexResultCallback(fromA, toA) {
    var allowedPenetration = 0f

    override fun addSingleResult(convexResult: CollisionWorld.LocalConvexResult, normalInWorldSpace: Boolean): Float {
        if (convexResult.hitCollisionObject === me) return 1f
        //ignore result if there is no contact response
        if (!convexResult.hitCollisionObject.hasContactResponse) return 1f
        val linVelA = convexToWorld - convexFromWorld
        val linVelB = Vec3()//toB.getOrigin()-fromB.getOrigin();
        val relativeVelocity = linVelA - linVelB
        //don't report time of impact for motion away from the contact normal (or causes minor penetration)
        if (convexResult.hitNormalLocal dot relativeVelocity >= -allowedPenetration) return 1f
        return super.addSingleResult(convexResult, normalInWorldSpace)
    }

    override fun needsCollision(proxy0: BroadphaseProxy): Boolean {
        // don't collide with itself
        if (proxy0.clientObject === me) return false
        // don't do CCD when the collision filters are not matching
        if (!super.needsCollision(proxy0)) return false
        val otherObj = proxy0.clientObject as CollisionObject
        //call needsResponse, see http://code.google.com/p/bullet/issues/detail?id=179
        return dispatcher.needsResponse(me, otherObj)
    }
}