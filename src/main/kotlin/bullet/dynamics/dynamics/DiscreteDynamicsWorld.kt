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

import bullet.BT_NO_PROFILE
import bullet.USE_STATIC_ONLY
import bullet.collision.broadphaseCollision.*
import bullet.collision.collisionDispatch.*
import bullet.collision.collisionShapes.SphereShape
import bullet.collision.narrowPhaseCollision.ManifoldPoint
import bullet.collision.narrowPhaseCollision.PersistentManifold
import bullet.dynamics.constraintSolver.ConstraintSolver
import bullet.dynamics.constraintSolver.ContactSolverInfo
import bullet.dynamics.constraintSolver.TypedConstraint
import bullet.gNumClampedCcdMotions
import bullet.i
import bullet.linearMath.*

/** DiscreteDynamicsWorld provides discrete rigid body simulation those classes replace the obsolete
 *  CcdPhysicsEnvironment/CcdPhysicsController  */
class DiscreteDynamicsWorld
/** this DiscreteDynamicsWorld constructor gets created objects from the user, and will not delete those */
constructor(dispatcher: Dispatcher?, pairCache: BroadphaseInterface, override var constraintSolver: ConstraintSolver,
            collisionConfiguration: CollisionConfiguration) : DynamicsWorld(dispatcher, pairCache, collisionConfiguration) {

    val sortedConstraints = ArrayList<TypedConstraint>()
    var solverIslandCallback: InplaceSolverIslandCallback? = null

    var islandManager: SimulationIslandManager? = null

    val constraints = ArrayList<TypedConstraint>()

    var nonStaticRigidBodies = ArrayList<RigidBody>()

    override var gravity = Vec3()
        set(value) {
            field put value
            nonStaticRigidBodies.filter { it.isActive && it.rigidbodyFlags hasnt RigidBodyFlags.DISABLE_WORLD_GRAVITY }
                    .forEach { it.gravity = gravity }
        }

    //for variable timesteps
    var localTime = 0f
    var fixedTimeStep = 0f
    //for variable timesteps

    var ownsIslandManager = false
    var ownsConstraintSolver = false
    var synchronizeAllMotionStates = false
    var applySpeculativeContactRestitution = false

    val actions = ArrayList<ActionInterface>()

    var profileTimings = 0

    var latencyMotionStateInterpolation = false

    val predictiveManifolds = ArrayList<PersistentManifold>()
    /** used to synchronize threads creating predictive contacts    */
    val predictiveManifoldsMutex = SpinMutex()

    fun predictUnconstraintMotion(timeStep: Float) {
//        BT_PROFILE("predictUnconstraintMotion");
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
//                    BT_PROFILE("CCD motion clamping")
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
//        BT_PROFILE("integrateTransforms");
        if (nonStaticRigidBodies.isNotEmpty())
            integrateTransformsInternal(nonStaticRigidBodies, nonStaticRigidBodies.size, timeStep)
        ///this should probably be switched on by default, but it is not well tested yet
        if (applySpeculativeContactRestitution) {
//            BT_PROFILE("apply speculative contact restitution");
            for (i in 0 until predictiveManifolds.size) {
                val manifold = predictiveManifolds[i]
                val body0 = RigidBody.upcast(manifold.body0!!)!!
                val body1 = RigidBody.upcast(manifold.body1!!)!!

                for (p in 0 until manifold.numContacts) {
                    val pt = manifold.getContactPoint(p)
                    val combinedRestitution = ManifoldResult.calculateCombinedRestitution(body0, body1)
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
//        BT_PROFILE("calculateSimulationIslands");
        islandManager!!.updateActivationState(this, dispatcher!!)

        //merge islands based on speculative contact manifolds too
        predictiveManifolds.forEach {
            val colObj0 = it.body0
            val colObj1 = it.body1

            if (colObj0 != null && !colObj0.isStaticOrKinematicObject && colObj1 != null && !colObj1.isStaticOrKinematicObject)
                islandManager!!.unionFind.unite(colObj0.islandTag, colObj1.islandTag)
        }
        constraints.filter { it.isEnabled }.forEach {
            val colObj0 = it.rbA
            val colObj1 = it.rbB
            if (colObj0 != null && !colObj0.isStaticOrKinematicObject && colObj1 != null && !colObj1.isStaticOrKinematicObject)
                islandManager!!.unionFind.unite(colObj0.islandTag, colObj1.islandTag)
        }
        //Store the island id in each body
        islandManager!!.storeIslandActivationState(this)
    }

    fun solveConstraints(solverInfo: ContactSolverInfo) {
//        BT_PROFILE("solveConstraints");
        sortedConstraints.resize(constraints.size)
        for (i in 0 until numConstraints)
            sortedConstraints[i] = constraints[i]
//	btAssert(0);
        sortedConstraints.sortWith(SortConstraintOnIslandPredicate)
        val constraintsPtr = sortedConstraints.takeIf { numConstraints != 0 }

        solverIslandCallback!!.setup(solverInfo, constraintsPtr!!, sortedConstraints.size, debugDrawer!!)
        constraintSolver.prepareSolve(numCollisionObjects, dispatcher!!.numManifolds)
        // solve all the constraints for this island
        islandManager!!.buildAndProcessIslands(dispatcher!!, this, solverIslandCallback!!)
        solverIslandCallback!!.processConstraints()
        constraintSolver.allSolved(solverInfo, debugDrawer!!)
    }

    fun updateActivationState(timeStep: Float) {
//        BT_PROFILE("updateActivationState");
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
//        BT_PROFILE("updateActions");
        actions.forEach { it.updateAction(this, timeStep) }
    }

    fun startProfiling(timeStep: Float) {
        if (!BT_NO_PROFILE)
            TODO()//CProfileManager::Reset();

    }

    fun internalSingleStepSimulation(timeStep: Float) {
//        BT_PROFILE("internalSingleStepSimulation");
        internalPreTickCallback?.invoke(this, timeStep)
        // apply gravity, predict motion
        predictUnconstraintMotion(timeStep)

        dispatchInfo.timeStep = timeStep
        dispatchInfo.stepCount = 0
        dispatchInfo.debugDraw = debugDrawer!!

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
//        BT_PROFILE( "release predictive contact manifolds" );
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
//                    BT_PROFILE("predictive convexSweepTest")
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
                                combinedFriction = ManifoldResult.calculateCombinedFriction(body, sweepResults.hitCollisionObject!!)
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
//        BT_PROFILE("createPredictiveContacts");
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
        //	BT_PROFILE("synchronizeMotionStates");
        if (synchronizeAllMotionStates)
        //iterate  over all collision objects
            collisionObjects.forEach {
                RigidBody.upcast(it)?.let { synchronizeSingleMotionState(it) }
            }
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
            body.motionState!!.worldTransform = interpolatedTransform
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

//    virtual void    addCollisionObject(btCollisionObject* collisionObject, int collisionFilterGroup=btBroadphaseProxy::StaticFilter, int collisionFilterMask=btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter)

    virtual void    addRigidBody(btRigidBody* body)

    virtual void    addRigidBody(btRigidBody* body, int group, int mask)

    virtual void    removeRigidBody(btRigidBody* body)

    ///removeCollisionObject will first check if it is a rigid body, if so call removeRigidBody otherwise call btCollisionWorld::removeCollisionObject
    virtual void    removeCollisionObject(btCollisionObject* collisionObject)


    virtual void    debugDrawConstraint(btTypedConstraint* constraint)

    virtual void    debugDrawWorld()

    virtual void    setConstraintSolver(btConstraintSolver* solver)

    virtual btConstraintSolver* getConstraintSolver()

    virtual    int        getNumConstraints() const

    virtual btTypedConstraint* getConstraint(int index)

    virtual const btTypedConstraint* getConstraint(int index) const


    virtual btDynamicsWorldType    getWorldType()
    const
    {
        return BT_DISCRETE_DYNAMICS_WORLD
    }

    ///the forces on each rigidbody is accumulating together with gravity. clear this after each timestep.
    virtual void    clearForces()

    /** apply gravity, call this once per timestep */
    fun applyGravity() {
        ///@todo: iterate over awake simulation islands!
        nonStaticRigidBodies.filter { it.isActive }.forEach { it.applyGravity() }
    }

    virtual void    setNumTasks(int numTasks)
    {
        (void) numTasks
    }

    ///obsolete, use updateActions instead
    virtual void updateVehicles(btScalar timeStep)
    {
        updateActions(timeStep)
    }

    ///obsolete, use addAction instead
    virtual void    addVehicle(btActionInterface* vehicle)
    ///obsolete, use removeAction instead
    virtual void    removeVehicle(btActionInterface* vehicle)
    ///obsolete, use addAction instead
    virtual void    addCharacter(btActionInterface* character)
    ///obsolete, use removeAction instead
    virtual void    removeCharacter(btActionInterface* character)

    void    setSynchronizeAllMotionStates(bool synchronizeAll)
    {
        m_synchronizeAllMotionStates = synchronizeAll
    }
    bool getSynchronizeAllMotionStates()
    const
    {
        return m_synchronizeAllMotionStates
    }

    void setApplySpeculativeContactRestitution(bool enable)
    {
        m_applySpeculativeContactRestitution = enable
    }

    bool getApplySpeculativeContactRestitution()
    const
    {
        return m_applySpeculativeContactRestitution
    }

    ///Preliminary serialization test for Bullet 2.76. Loading those files requires a separate parser (see Bullet/Demos/SerializeDemo)
    virtual    void    serialize(btSerializer* serializer)

    ///Interpolate motion state between previous and current transform, instead of current and next transform.
    ///This can relieve discontinuities in the rendering, due to penetrations
    void setLatencyMotionStateInterpolation(bool latencyInterpolation )
    {
        m_latencyMotionStateInterpolation = latencyInterpolation
    }
    bool getLatencyMotionStateInterpolation()
    const
    {
        return m_latencyMotionStateInterpolation
    }
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

    fun setup(solverInfo: ContactSolverInfo, sortedConstraints: ArrayList<TypedConstraint>, numConstraints: Int,
              debugDrawer: DebugDraw) {
//        assert(solverInfo != null)
        this.solverInfo = solverInfo
        with(this.sortedConstraints) { clear(); addAll(sortedConstraints) }
        this.numConstraints = numConstraints
        this.debugDrawer = debugDrawer
    }

    override fun processIsland(bodies: ArrayList<CollisionObject>, numBodies: Int, manifolds: MutableList<PersistentManifold>,
                               numManifolds: Int, islandId: Int) {
        if (islandId < 0)
        ///we don't split islands, so all constraints/contact manifolds/bodies are passed into the solver regardless the island id
            solver.solveGroup(bodies, numBodies, manifolds, numManifolds, sortedConstraints, numConstraints, solverInfo!!,
                    debugDrawer!!, dispatcher)
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
                solver.solveGroup(bodies, numBodies, manifolds, numManifolds,
                        sortedConstraints.subList(startConstraint, sortedConstraints.size), numCurConstraints, solverInfo!!,
                        debugDrawer!!, dispatcher)
            else {
                for (j in 0 until numBodies) bodies.add(bodies[j])
                for (j in 0 until numManifolds) manifolds.add(manifolds[j])
                for (j in 0 until numCurConstraints) constraints.add(sortedConstraints[startConstraint + j])
                if ((constraints.size + manifolds.size) > solverInfo!!.minimumSolverBatchSize)
                    processConstraints()
                else println("deferred")
            }
        }
    }

    fun processConstraints() {
        solver.solveGroup(bodies, bodies.size, manifolds, manifolds.size, constraints, constraints.size, solverInfo!!, debugDrawer!!,
                dispatcher)
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