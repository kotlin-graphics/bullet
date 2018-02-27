/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

package bullet.collision.collisionDispatch

import bullet.*
import bullet.collision.broadphaseCollision.*
import bullet.collision.collisionShapes.*
import bullet.collision.narrowPhaseCollision.*
import bullet.linearMath.*
import bullet.collision.broadphaseCollision.BroadphaseNativeTypes as Bnt
import bullet.collision.broadphaseCollision.DispatcherQueryType as Dqt
import bullet.collision.collisionDispatch.CollisionObject.CollisionObjectTypes as Cot

/**
 * @mainpage Bullet Documentation
 *
 * @section intro_sec Introduction
 * Bullet is a Collision Detection and Rigid Body Dynamics Library. The Library is Open Source and free for commercial use, under the ZLib license ( http://opensource.org/licenses/zlib-license.php ).
 *
 * The main documentation is Bullet_User_Manual.pdf, included in the source code distribution.
 * There is the Physics Forum for feedback and general Collision Detection and Physics discussions.
 * Please visit http://www.bulletphysics.org
 *
 * @section install_sec Installation
 *
 * @subsection step1 Step 1: Download
 * You can download the Bullet Physics Library from the github repository: https://github.com/bulletphysics/bullet3/releases
 *
 * @subsection step2 Step 2: Building
 * Bullet has multiple build systems, including premake, cmake and autotools. Premake and cmake support all platforms.
 * Premake is included in the Bullet/build folder for Windows, Mac OSX and Linux.
 * Under Windows you can click on Bullet/build/vs2010.bat to create Microsoft Visual Studio projects.
 * On Mac OSX and Linux you can open a terminal and generate Makefile, codeblocks or Xcode4 projects:
 * cd Bullet/build
 * ./premake4_osx gmake or ./premake4_linux gmake or ./premake4_linux64 gmake or (for Mac) ./premake4_osx xcode4
 * cd Bullet/build/gmake
 * make
 *
 * An alternative to premake is cmake. You can download cmake from http://www.cmake.org
 * cmake can autogenerate projectfiles for Microsoft Visual Studio, Apple Xcode, KDevelop and Unix Makefiles.
 * The easiest is to run the CMake cmake-gui graphical user interface and choose the options and generate projectfiles.
 * You can also use cmake in the command-line. Here are some examples for various platforms:
 * cmake . -G "Visual Studio 9 2008"
 * cmake . -G Xcode
 * cmake . -G "Unix Makefiles"
 * Although cmake is recommended, you can also use autotools for UNIX: ./autogen.sh ./configure to create a Makefile and then run make.
 *
 * @subsection step3 Step 3: Testing demos
 * Try to run and experiment with BasicDemo executable as a starting point.
 * Bullet can be used in several ways, as Full Rigid Body simulation, as Collision Detector Library or Low Level / Snippets like the GJK Closest Point calculation.
 * The Dependencies can be seen in this documentation under Directories
 *
 * @subsection step4 Step 4: Integrating in your application, full Rigid Body and Soft Body simulation
 * Check out BasicDemo how to create a btDynamicsWorld, btRigidBody and btCollisionShape, Stepping the simulation and synchronizing your graphics object transform.
 * Check out SoftDemo how to use soft body dynamics, using btSoftRigidDynamicsWorld.
 * @subsection step5 Step 5 : Integrate the Collision Detection Library (without Dynamics and other Extras)
 * Bullet Collision Detection can also be used without the Dynamics/Extras.
 * Check out btCollisionWorld and btCollisionObject, and the CollisionInterfaceDemo.
 * @subsection step6 Step 6 : Use Snippets like the GJK Closest Point calculation.
 * Bullet has been designed in a modular way keeping dependencies to a minimum. The ConvexHullDistance demo demonstrates direct use of btGjkPairDetector.
 *
 * @section copyright Copyright
 * For up-to-data information and copyright and contributors list check out the Bullet_User_Manual.pdf
 *
 */

/** CollisionWorld is interface and container for the collision detection   */
open class CollisionWorld
/** this constructor doesn't own the dispatcher and paircache/broadphase    */
(
        var dispatcher: Dispatcher?,
        val broadphasePairCache: BroadphaseInterface,
        collisionConfiguration: CollisionConfiguration
) {

    val collisionObjects = ArrayList<CollisionObject>()

    var dispatchInfo = DispatcherInfo()

    var debugDrawer: DebugDraw? = null

    /** forceUpdateAllAabbs can be set to false as an optimization to only update active object AABBs
     *  it is true by default, because it is error-prone (setting the position of static objects wouldn't update their AABB)    */
    var forceUpdateAllAabbs = true

    val broadphase get() = broadphasePairCache

    val pairCache get() = broadphasePairCache.overlappingPairCache

    fun updateSingleAabb(colObj: CollisionObject) {

        val minAabb = Vec3()
        val maxAabb = Vec3()
        colObj.collisionShape!!.getAabb(colObj.getWorldTransform(), minAabb, maxAabb)
        // need to increase the aabb for contact thresholds
        val contactThreshold = gContactBreakingThreshold
        minAabb -= contactThreshold
        maxAabb += contactThreshold

        if (dispatchInfo.useContinuous && colObj.internalType == Cot.RIGID_BODY.i && !colObj.isStaticOrKinematicObject) {
            val minAabb2 = Vec3()
            val maxAabb2 = Vec3()
            colObj.collisionShape!!.getAabb(colObj.getInterpolationWorldTransform(), minAabb2, maxAabb2)
            minAabb2 -= contactThreshold
            maxAabb2 += contactThreshold
            minAabb.setMin(minAabb2)
            maxAabb.setMax(maxAabb2)
        }

        val bp = broadphasePairCache

        //moving objects should be moderately sized, probably something wrong if not
        if (colObj.isStaticObject || (maxAabb - minAabb).length2() < 1e12f)
            bp.setAabb(colObj.broadphaseHandle!!, minAabb, maxAabb, dispatcher!!)
        else {
            /*  something went wrong, investigate
                this assert is unwanted in 3D modelers (danger of loosing work)             */
            colObj.activationState = DISABLE_SIMULATION

            debugDrawer?.let {
                // TODO
//                reportErrorWarning("Overflow in AABB, object removed from simulation")
//                reportErrorWarning("If you can reproduce this, please email bugs@continuousphysics.com\n")
//                reportErrorWarning("Please include above information, your Platform, version of OS.\n")
//                reportErrorWarning("Thanks.\n")
            }
            throw Error()   // tmp
        }
    }

    fun updateAabbs() {
        for (i in 0 until collisionObjects.size) {
            val colObj = collisionObjects[i]
            assert(colObj.worldArrayIndex == i)
            //only update aabb of active objects
            if (forceUpdateAllAabbs || colObj.isActive) updateSingleAabb(colObj)
        }
    }

    /** The computeOverlappingPairs is usually already called by performDiscreteCollisionDetection (or stepSimulation)
     *  it can be useful to use if you perform ray tests without collision detection/simulation */
    fun computeOverlappingPairs() {
        BT_PROFILE("calculateOverlappingPairs")
        broadphasePairCache.calculateOverlappingPairs(dispatcher!!)
    }

    open fun debugDrawWorld() {

        debugDrawer?.let {

            //            it.clearLines(); TODO
//
//            btIDebugDraw::DefaultColors defaultColors = getDebugDrawer()->getDefaultColors();
//
//            if ( getDebugDrawer()->getDebugMode() & btIDebugDraw::DBG_DrawContactPoints)
//            {
//
//
//                if (getDispatcher())
//                {
//                    int numManifolds = getDispatcher()->getNumManifolds();
//
//                    for (int i=0;i<numManifolds;i++)
//                    {
//                        btPersistentManifold* contactManifold = getDispatcher()->getManifoldByIndexInternal(i);
//                        //btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
//                        //btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
//
//                        int numContacts = contactManifold->getNumContacts();
//                        for (int j=0;j<numContacts;j++)
//                        {
//                            btManifoldPoint& cp = contactManifold->getContactPoint(j);
//                            getDebugDrawer()->drawContactPoint(cp.m_positionWorldOnB,cp.m_normalWorldOnB,cp.getDistance(),cp.getLifeTime(),defaultColors.m_contactPoint);
//                        }
//                    }
//                }
//            }
//
//            if ((getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawAabb)))
//            {
//                int i;
//
//                for (  i=0;i<m_collisionObjects.size();i++)
//                {
//                    btCollisionObject* colObj = m_collisionObjects[i];
//                    if ((colObj->getCollisionFlags() & btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT)==0)
//                    {
//                        if (getDebugDrawer() && (getDebugDrawer()->getDebugMode() & btIDebugDraw::DBG_DrawWireframe))
//                        {
//                            btVector3 color(btScalar(0.4),btScalar(0.4),btScalar(0.4));
//
//                            switch(colObj->getActivationState())
//                            {
//                                case  ACTIVE_TAG:
//                                color = defaultColors.m_activeObject; break;
//                                case ISLAND_SLEEPING:
//                                color =  defaultColors.m_deactivatedObject;break;
//                                case WANTS_DEACTIVATION:
//                                color = defaultColors.m_wantsDeactivationObject;break;
//                                case DISABLE_DEACTIVATION:
//                                color = defaultColors.m_disabledDeactivationObject;break;
//                                case DISABLE_SIMULATION:
//                                color = defaultColors.m_disabledSimulationObject;break;
//                                default:
//                                {
//                                    color = btVector3(btScalar(.3),btScalar(0.3),btScalar(0.3));
//                                }
//                            };
//
//                            colObj->getCustomDebugColor(color);
//
//                            debugDrawObject(colObj->getWorldTransform(),colObj->getCollisionShape(),color);
//                        }
//                        if (m_debugDrawer && (m_debugDrawer->getDebugMode() & btIDebugDraw::DBG_DrawAabb))
//                        {
//                            btVector3 minAabb,maxAabb;
//                            btVector3 colorvec = defaultColors.m_aabb;
//                            colObj->getCollisionShape()->getAabb(colObj->getWorldTransform(), minAabb,maxAabb);
//                            btVector3 contactThreshold(gContactBreakingThreshold,gContactBreakingThreshold,gContactBreakingThreshold);
//                            minAabb -= contactThreshold;
//                            maxAabb += contactThreshold;
//
//                            btVector3 minAabb2,maxAabb2;
//
//                            if(getDispatchInfo().m_useContinuous && colObj->getInternalType()==btCollisionObject::CO_RIGID_BODY && !colObj->isStaticOrKinematicObject())
//                            {
//                                colObj->getCollisionShape()->getAabb(colObj->getInterpolationWorldTransform(),minAabb2,maxAabb2);
//                                minAabb2 -= contactThreshold;
//                                maxAabb2 += contactThreshold;
//                                minAabb.setMin(minAabb2);
//                                maxAabb.setMax(maxAabb2);
//                            }
//
//                            m_debugDrawer->drawAabb(minAabb,maxAabb,colorvec);
//                        }
//                    }
//                }
//            }
        }
    }

    fun debugDrawObject(worldTransform: Transform, shape: CollisionShape, color: Vec3) {

        // Draw a small simplex at the center of the object TODO
//        debugDrawer?.let {
//            if(it..getDebugMode() & btIDebugDraw::DBG_DrawFrames)
//            getDebugDrawer()->drawTransform(worldTransform,.1);
//        }
//
//        if (shape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE)
//        {
//            const btCompoundShape* compoundShape = static_cast<const btCompoundShape*>(shape);
//            for (int i=compoundShape->getNumChildShapes()-1;i>=0;i--)
//            {
//                btTransform childTrans = compoundShape->getChildTransform(i);
//                const btCollisionShape* colShape = compoundShape->getChildShape(i);
//                debugDrawObject(worldTransform*childTrans,colShape,color);
//            }
//
//        } else
//        {
//
//            switch (shape->getShapeType())
//            {
//
//                case BOX_SHAPE_PROXYTYPE:
//                {
//                    const btBoxShape* boxShape = static_cast<const btBoxShape*>(shape);
//                    btVector3 halfExtents = boxShape->getHalfExtentsWithMargin();
//                    getDebugDrawer()->drawBox(-halfExtents,halfExtents,worldTransform,color);
//                    break;
//                }
//
//                case SPHERE_SHAPE_PROXYTYPE:
//                {
//                    const btSphereShape* sphereShape = static_cast<const btSphereShape*>(shape);
//                    btScalar radius = sphereShape->getMargin();//radius doesn't include the margin, so draw with margin
//
//                    getDebugDrawer()->drawSphere(radius, worldTransform, color);
//                    break;
//                }
//                case MULTI_SPHERE_SHAPE_PROXYTYPE:
//                {
//                    const btMultiSphereShape* multiSphereShape = static_cast<const btMultiSphereShape*>(shape);
//
//                    btTransform childTransform;
//                    childTransform.setIdentity();
//
//                    for (int i = multiSphereShape->getSphereCount()-1; i>=0;i--)
//                    {
//                        childTransform.setOrigin(multiSphereShape->getSpherePosition(i));
//                        getDebugDrawer()->drawSphere(multiSphereShape->getSphereRadius(i), worldTransform*childTransform, color);
//                    }
//
//                    break;
//                }
//                case CAPSULE_SHAPE_PROXYTYPE:
//                {
//                    const btCapsuleShape* capsuleShape = static_cast<const btCapsuleShape*>(shape);
//
//                    btScalar radius = capsuleShape->getRadius();
//                    btScalar halfHeight = capsuleShape->getHalfHeight();
//
//                    int upAxis = capsuleShape->getUpAxis();
//                    getDebugDrawer()->drawCapsule(radius, halfHeight, upAxis, worldTransform, color);
//                    break;
//                }
//                case CONE_SHAPE_PROXYTYPE:
//                {
//                    const btConeShape* coneShape = static_cast<const btConeShape*>(shape);
//                    btScalar radius = coneShape->getRadius();//+coneShape->getMargin();
//                    btScalar height = coneShape->getHeight();//+coneShape->getMargin();
//
//                    int upAxis= coneShape->getConeUpIndex();
//                    getDebugDrawer()->drawCone(radius, height, upAxis, worldTransform, color);
//                    break;
//
//                }
//                case CYLINDER_SHAPE_PROXYTYPE:
//                {
//                    const btCylinderShape* cylinder = static_cast<const btCylinderShape*>(shape);
//                    int upAxis = cylinder->getUpAxis();
//                    btScalar radius = cylinder->getRadius();
//                    btScalar halfHeight = cylinder->getHalfExtentsWithMargin()[upAxis];
//                    getDebugDrawer()->drawCylinder(radius, halfHeight, upAxis, worldTransform, color);
//                    break;
//                }
//
//                case STATIC_PLANE_PROXYTYPE:
//                {
//                    const btStaticPlaneShape* staticPlaneShape = static_cast<const btStaticPlaneShape*>(shape);
//                    btScalar planeConst = staticPlaneShape->getPlaneConstant();
//                    const btVector3& planeNormal = staticPlaneShape->getPlaneNormal();
//                    getDebugDrawer()->drawPlane(planeNormal, planeConst,worldTransform, color);
//                    break;
//
//                }
//                default:
//                {
//
//                    /// for polyhedral shapes
//                    if (shape->isPolyhedral())
//                    {
//                        btPolyhedralConvexShape* polyshape = (btPolyhedralConvexShape*) shape;
//
//                        int i;
//                        if (polyshape->getConvexPolyhedron())
//                        {
//                            const btConvexPolyhedron* poly = polyshape->getConvexPolyhedron();
//                            for (i=0;i<poly->m_faces.size();i++)
//                            {
//                                btVector3 centroid(0,0,0);
//                                int numVerts = poly->m_faces[i].m_indices.size();
//                                if (numVerts)
//                                {
//                                    int lastV = poly->m_faces[i].m_indices[numVerts-1];
//                                    for (int v=0;v<poly->m_faces[i].m_indices.size();v++)
//                                    {
//                                        int curVert = poly->m_faces[i].m_indices[v];
//                                        centroid+=poly->m_vertices[curVert];
//                                        getDebugDrawer()->drawLine(worldTransform*poly->m_vertices[lastV],worldTransform*poly->m_vertices[curVert],color);
//                                        lastV = curVert;
//                                    }
//                                }
//                                centroid*= btScalar(1.f)/btScalar(numVerts);
//                                if (getDebugDrawer()->getDebugMode() & btIDebugDraw::DBG_DrawNormals)
//                                {
//                                    btVector3 normalColor(1,1,0);
//                                    btVector3 faceNormal(poly->m_faces[i].m_plane[0],poly->m_faces[i].m_plane[1],poly->m_faces[i].m_plane[2]);
//                                    getDebugDrawer()->drawLine(worldTransform*centroid,worldTransform*(centroid+faceNormal),normalColor);
//                                }
//
//                            }
//
//
//                        } else
//                        {
//                            for (i=0;i<polyshape->getNumEdges();i++)
//                            {
//                                btVector3 a,b;
//                                polyshape->getEdge(i,a,b);
//                                btVector3 wa = worldTransform * a;
//                                btVector3 wb = worldTransform * b;
//                                getDebugDrawer()->drawLine(wa,wb,color);
//                            }
//                        }
//
//
//                    }
//
//                    if (shape->isConcave())
//                    {
//                        btConcaveShape* concaveMesh = (btConcaveShape*) shape;
//
//                        ///@todo pass camera, for some culling? no -> we are not a graphics lib
//                        btVector3 aabbMax(btScalar(BT_LARGE_FLOAT),btScalar(BT_LARGE_FLOAT),btScalar(BT_LARGE_FLOAT));
//                        btVector3 aabbMin(btScalar(-BT_LARGE_FLOAT),btScalar(-BT_LARGE_FLOAT),btScalar(-BT_LARGE_FLOAT));
//
//                        DebugDrawcallback drawCallback(getDebugDrawer(),worldTransform,color);
//                        concaveMesh->processAllTriangles(&drawCallback,aabbMin,aabbMax);
//
//                    }
//
//                    if (shape->getShapeType() == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE)
//                    {
//                        btConvexTriangleMeshShape* convexMesh = (btConvexTriangleMeshShape*) shape;
//                        //todo: pass camera for some culling
//                        btVector3 aabbMax(btScalar(BT_LARGE_FLOAT),btScalar(BT_LARGE_FLOAT),btScalar(BT_LARGE_FLOAT));
//                        btVector3 aabbMin(btScalar(-BT_LARGE_FLOAT),btScalar(-BT_LARGE_FLOAT),btScalar(-BT_LARGE_FLOAT));
//                        //DebugDrawcallback drawCallback;
//                        DebugDrawcallback drawCallback(getDebugDrawer(),worldTransform,color);
//                        convexMesh->getMeshInterface()->InternalProcessAllTriangles(&drawCallback,aabbMin,aabbMax);
//                    }
//
//
//
//                }
//
//            }
//        }
    }

    /** LocalShapeInfo gives extra information for complex shapes
     *  Currently, only TriangleMeshShape is available, so it just contains triangleIndex and subpart   */
    class LocalShapeInfo(val shapePart: Int, val triangleIndex: Int)

    class LocalRayResult(val collisionObject: CollisionObject?, var localShapeInfo: LocalShapeInfo?, val hitNormalLocal: Vec3, val hitFraction: Float)

    /** RayResultCallback is used to report new raycast results */
    abstract class RayResultCallback {
        var closestHitFraction = 1f
        var collisionObject: CollisionObject? = null
        var collisionFilterGroup = BroadphaseProxy.CollisionFilterGroups.DefaultFilter.i
        var collisionFilterMask = BroadphaseProxy.CollisionFilterGroups.AllFilter.i
        /** @BP Mod - Custom flags, currently used to enable backface culling on tri-meshes, see RaycastCallback.
         *  Apply any of the EFlags defined there on flags here to invoke.  */
        var flags = 0

        val hasHit get() = collisionObject != null

        open infix fun needsCollision(proxy0: BroadphaseProxy) = proxy0.collisionFilterGroup has collisionFilterMask && collisionFilterGroup has proxy0.collisionFilterMask

        abstract fun addSingleResult(rayResult: LocalRayResult, normalInWorldSpace: Boolean): Float
    }

    class ClosestRayResultCallback(
            /** used to calculate hitPointWorld from hitFraction    */
            val rayFromWorld: Vec3,
            val rayToWorld: Vec3) : RayResultCallback() {

        val hitNormalWorld = Vec3()
        val hitPointWorld = Vec3()

        override fun addSingleResult(rayResult: LocalRayResult, normalInWorldSpace: Boolean): Float {
            // caller already does the filter on the m_closestHitFraction
            assert(rayResult.hitFraction <= closestHitFraction)

            closestHitFraction = rayResult.hitFraction
            collisionObject = rayResult.collisionObject
            if (normalInWorldSpace) hitNormalWorld put rayResult.hitNormalLocal
            else // need to transform normal into worldspace
                hitNormalWorld put collisionObject!!.getWorldTransform().basis * rayResult.hitNormalLocal
            hitPointWorld.setInterpolate3(rayFromWorld, rayToWorld, rayResult.hitFraction)
            return rayResult.hitFraction
        }
    }

    class AllHitsRayResultCallback(
            /** used to calculate hitPointWorld from hitFraction    */
            val rayFromWorld: Vec3, val rayToWorld: Vec3) : RayResultCallback() {

        val collisionObjects = ArrayList<CollisionObject>()

        val hitNormalWorld = ArrayList<Vec3>()
        val hitPointWorld = ArrayList<Vec3>()
        val hitFractions = ArrayList<Float>()

        override fun addSingleResult(rayResult: LocalRayResult, normalInWorldSpace: Boolean): Float {

            collisionObject = rayResult.collisionObject
            collisionObjects.add(rayResult.collisionObject!!)
            val hitNormalWorld = Vec3()
            if (normalInWorldSpace)
                hitNormalWorld put rayResult.hitNormalLocal
            else    // need to transform normal into worldspace
                hitNormalWorld put collisionObject!!.getWorldTransform().basis * rayResult.hitNormalLocal
            this.hitNormalWorld.add(hitNormalWorld)
            val hitPointWorld = Vec3()
            hitPointWorld.setInterpolate3(rayFromWorld, rayToWorld, rayResult.hitFraction)
            this.hitPointWorld.add(hitPointWorld)
            hitFractions.add(rayResult.hitFraction)
            return closestHitFraction
        }
    }

    class LocalConvexResult(val hitCollisionObject: CollisionObject, var localShapeInfo: LocalShapeInfo?,
                            val hitNormalLocal: Vec3, val hitPointLocal: Vec3, val hitFraction: Float)

    /** RayResultCallback is used to report new raycast results */
    abstract class ConvexResultCallback {

        var closestHitFraction = 1f
        var collisionFilterGroup = BroadphaseProxy.CollisionFilterGroups.DefaultFilter.i
        var collisionFilterMask = BroadphaseProxy.CollisionFilterGroups.AllFilter.i

        val hasHit get() = closestHitFraction < 1f

        infix open fun needsCollision(proxy0: BroadphaseProxy) = proxy0.collisionFilterGroup has collisionFilterMask && collisionFilterGroup has proxy0.collisionFilterMask

        abstract fun addSingleResult(convexResult: LocalConvexResult, normalInWorldSpace: Boolean): Float
    }

    open class ClosestConvexResultCallback(
            /** used to calculate hitPointWorld from hitFraction    */
            val convexFromWorld: Vec3, val convexToWorld: Vec3) : ConvexResultCallback() {

        val hitNormalWorld = Vec3()
        val hitPointWorld = Vec3()
        var hitCollisionObject: CollisionObject? = null

        override fun addSingleResult(convexResult: LocalConvexResult, normalInWorldSpace: Boolean): Float {
            //caller already does the filter on the m_closestHitFraction
            assert(convexResult.hitFraction <= closestHitFraction)

            closestHitFraction = convexResult.hitFraction
            hitCollisionObject = convexResult.hitCollisionObject
            if (normalInWorldSpace)
                hitNormalWorld put convexResult.hitNormalLocal
            else // need to transform normal into worldspace
                hitNormalWorld put hitCollisionObject!!.getWorldTransform().basis * convexResult.hitNormalLocal
            hitPointWorld put convexResult.hitPointLocal
            return convexResult.hitFraction
        }
    }

    // ContactResultCallback is used to report contact points
    abstract class ContactResultCallback {
        var collisionFilterGroup = BroadphaseProxy.CollisionFilterGroups.DefaultFilter.i
        var collisionFilterMask = BroadphaseProxy.CollisionFilterGroups.AllFilter.i
        var closestDistanceThreshold = 0f

        infix fun needsCollision(proxy0: BroadphaseProxy) = proxy0.collisionFilterGroup has collisionFilterMask && collisionFilterGroup has proxy0.collisionFilterMask

        abstract fun addSingleResult(cp: ManifoldPoint, colObj0Wrap: CollisionObjectWrapper?, partId0: Int, index0: Int,
                                     colObj1Wrap: CollisionObjectWrapper?, partId1: Int, index1: Int): Float
    }

    val numCollisionObjects get() = collisionObjects.size

    /** rayTest performs a raycast on all objects in the btCollisionWorld, and calls the resultCallback
     *  This allows for several queries: first hit, all hits, any hit, dependent on the value returned by the callback. */
    fun rayTest(rayFromWorld: Vec3, rayToWorld: Vec3, resultCallback: RayResultCallback) {
        /*  use the broadphase to accelerate the search for objects, based on their aabb and for each object with
            ray-aabb overlap, perform an exact ray test         */
        val rayCB = SingleRayCallback(rayFromWorld, rayToWorld, this, resultCallback)

        broadphasePairCache.rayTest(rayFromWorld, rayToWorld, rayCB)
    }

    /** convexTest performs a swept convex cast on all objects in the CollisionWorld, and calls the resultCallback
     *  This allows for several queries: first hit, all hits, any hit, dependent on the value return by the callback.   */
    fun convexSweepTest(castShape: ConvexShape, convexFromWorld: Transform, convexToWorld: Transform,
                        resultCallback: ConvexResultCallback, allowedCcdPenetration: Float = 0f) {

        /*  use the broadphase to accelerate the search for objects, based on their aabb and for each object with
            ray-aabb overlap, perform an exact ray test
            unfortunately the implementation for rayTest and convexSweepTest duplicated, albeit practically identical   */
        val convexFromTrans = convexFromWorld
        val convexToTrans = convexToWorld
        val castShapeAabbMin = Vec3()
        val castShapeAabbMax = Vec3()
        /* Compute AABB that encompasses angular movement */
        val linVel = Vec3()
        val angVel = Vec3()
        TransformUtil.calculateVelocity(convexFromTrans, convexToTrans, 1f, linVel, angVel)
        val zeroLinVel = Vec3()
        val r = Transform().apply {
            setIdentity()
            setRotation(convexFromTrans.getRotation())
        }
        castShape.calculateTemporalAabb(r, zeroLinVel, angVel, 1f, castShapeAabbMin, castShapeAabbMax)

        val convexCB = SingleSweepCallback(castShape, convexFromWorld, convexToWorld, this, resultCallback, allowedCcdPenetration)

        broadphasePairCache.rayTest(convexFromTrans.origin, convexToTrans.origin, convexCB, castShapeAabbMin, castShapeAabbMax)
    }

    /** contactTest performs a discrete collision test between colObj against all objects in the CollisionWorld, and calls the resultCallback.
     *  It reports one or more contact points for every overlapping object (including the one with deepest penetration) */
    fun contactTest(colObj: CollisionObject, resultCallback: ContactResultCallback) {
        val aabbMin = Vec3()
        val aabbMax = Vec3()
        colObj.collisionShape!!.getAabb(colObj.getWorldTransform(), aabbMin, aabbMax)
        val contactCB = SingleContactCallback(colObj, this, resultCallback)

        broadphasePairCache.aabbTest(aabbMin, aabbMax, contactCB)
    }

    /** ContactTest performs a discrete collision test between two collision objects and calls the resultCallback if overlap if detected.
     *  It reports one or more contact points (including the one with deepest penetration)  */
    fun contactPairTest(colObjA: CollisionObject, colObjB: CollisionObject, resultCallback: ContactResultCallback) {
        val obA = CollisionObjectWrapper(null, colObjA.collisionShape, colObjA, colObjA.getWorldTransform(), -1, -1)
        val obB = CollisionObjectWrapper(null, colObjB.collisionShape, colObjB, colObjB.getWorldTransform(), -1, -1)

        dispatcher!!.findAlgorithm(obA, obB, null, Dqt.CLOSEST_POINT_ALGORITHMS)?.let { algorithm ->
            val contactPointResult = BridgedManifoldResult(obA, obB, resultCallback)
            contactPointResult.closestPointDistanceThreshold = resultCallback.closestDistanceThreshold
            // discrete collision detection query
            algorithm.processCollision(obA, obB, dispatchInfo, contactPointResult)

            dispatcher!!.freeCollisionAlgorithm(algorithm)
        }
    }

    companion object {

        /** rayTestSingle performs a raycast call and calls the resultCallback. It is used internally by rayTest.
         *  In a future implementation, we consider moving the ray test as a virtual method in CollisionShape.
         *  This allows more customization. */
        fun rayTestSingle(rayFromTrans: Transform, rayToTrans: Transform, collisionObject: CollisionObject,
                          collisionShape: CollisionShape, colObjWorldTransform: Transform, resultCallback: RayResultCallback) {

            val colObWrap = CollisionObjectWrapper(null, collisionShape, collisionObject, colObjWorldTransform, -1, -1)
            rayTestSingleInternal(rayFromTrans, rayToTrans, colObWrap, resultCallback)
        }

        fun rayTestSingleInternal(rayFromTrans: Transform, rayToTrans: Transform, collisionObjectWrap: CollisionObjectWrapper,
                                  resultCallback: RayResultCallback) {

            val pointShape = SphereShape(0f)
            pointShape.margin = 0f
            val castShape = pointShape as ConvexShape
            val collisionShape = collisionObjectWrap.collisionShape
            val colObjWorldTransform = collisionObjectWrap.worldTransform

            if (collisionShape.isConvex) {
                BT_PROFILE("rayTestConvex")
                val castResult = ConvexCast.CastResult()
                castResult.fraction = resultCallback.closestHitFraction

                val convexShape = collisionShape as ConvexShape
                val simplexSolver = VoronoiSimplexSolver()
                val subSimplexConvexCaster = SubsimplexConvexCast(castShape, convexShape, simplexSolver)

                val gjkConvexCaster = GjkConvexCast(castShape, convexShape, simplexSolver)

                //btContinuousConvexCollision convexCaster(castShape,convexShape,&simplexSolver,0);

                var convexCaster: ConvexCast? = null
                // use kF_UseSubSimplexConvexCastRaytest by default
                convexCaster = if (resultCallback.flags has TriangleRaycastCallback.Flags.UseGjkConvexCastRaytest.i)
                    gjkConvexCaster
                else subSimplexConvexCaster

                if (convexCaster.calcTimeOfImpact(rayFromTrans, rayToTrans, colObjWorldTransform, colObjWorldTransform, castResult))
                //add hit
                    if (castResult.normal.length2() > 0.0001f)
                        if (castResult.fraction < resultCallback.closestHitFraction) {
                            //todo: figure out what this is about. When is rayFromTest.getBasis() not identity?
                            castResult.normal.normalize()
                            val localRayResult = CollisionWorld.LocalRayResult(collisionObjectWrap.collisionObject!!,
                                    null, castResult.normal, castResult.fraction)
                            val normalInWorldSpace = true
                            resultCallback.addSingleResult(localRayResult, normalInWorldSpace)
                        }
            } else if (collisionShape.isConcave) {

                // ConvexCast::CastResult
                class BridgeTriangleRaycastCallback(from: Vec3, to: Vec3, val resultCallback: CollisionWorld.RayResultCallback,
                                                    val collisionObject: CollisionObject?, val triangleMesh: ConcaveShape,
                                                    val colObjWorldTransform: Transform)
                    : TriangleRaycastCallback(from, to, resultCallback.flags) {

                    override fun reportHit(hitNormalLocal: Vec3, hitFraction: Float, partId: Int, triangleIndex: Int): Float {
                        val shapeInfo = CollisionWorld.LocalShapeInfo(partId, triangleIndex)
                        val hitNormalWorld = colObjWorldTransform.basis * hitNormalLocal
                        val rayResult = CollisionWorld.LocalRayResult(collisionObject, shapeInfo, hitNormalWorld, hitFraction)
                        val normalInWorldSpace = true
                        return resultCallback.addSingleResult(rayResult, normalInWorldSpace)
                    }
                }

                val worldTocollisionObject = colObjWorldTransform.inverse()
                val rayFromLocal = worldTocollisionObject * rayFromTrans.origin
                val rayToLocal = worldTocollisionObject * rayToTrans.origin

                BT_PROFILE("rayTestConcave")
                when (collisionShape.shapeType) {
                    Bnt.TRIANGLE_MESH_SHAPE_PROXYTYPE -> {
                        // optimized version for btBvhTriangleMeshShape
                        val triangleMesh = collisionShape as BvhTriangleMeshShape

                        val rcb = BridgeTriangleRaycastCallback(rayFromLocal, rayToLocal, resultCallback,
                                collisionObjectWrap.collisionObject, triangleMesh, colObjWorldTransform)
                        rcb.hitFraction = resultCallback.closestHitFraction
                        triangleMesh.performRaycast(rcb, rayFromLocal, rayToLocal)
                    }
                    Bnt.SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE -> {
                        // optimized version for ScaledBvhTriangleMeshShape
                        val scaledTriangleMesh = collisionShape as ScaledBvhTriangleMeshShape
                        val triangleMesh = scaledTriangleMesh.childShape as BvhTriangleMeshShape

                        // scale the ray positions
                        val scale = scaledTriangleMesh.localScaling_
                        val rayFromLocalScaled = rayFromLocal / scale
                        val rayToLocalScaled = rayToLocal / scale

                        // perform raycast in the underlying btBvhTriangleMeshShape
                        val rcb = BridgeTriangleRaycastCallback (rayFromLocalScaled, rayToLocalScaled, resultCallback,
                                collisionObjectWrap.collisionObject, triangleMesh, colObjWorldTransform)
                        rcb.hitFraction = resultCallback.closestHitFraction
                        triangleMesh.performRaycast(rcb, rayFromLocalScaled, rayToLocalScaled)
                    }

                    else -> {
                        //generic (slower) case
                        val concaveShape = collisionShape as ConcaveShape

                        class BridgeTriangleRaycastCallback_(from: Vec3, to: Vec3, val resultCallback: RayResultCallback,
                                                             val collisionObject: CollisionObject, val triangleMesh: ConcaveShape,
                                                             val colObjWorldTransform: Transform) :
                                TriangleRaycastCallback(from, to, resultCallback.flags) {

                            override fun reportHit(hitNormalLocal: Vec3, hitFraction: Float, partId: Int, triangleIndex: Int): Float {

                                val shapeInfo = CollisionWorld.LocalShapeInfo(partId, triangleIndex)
                                val hitNormalWorld = colObjWorldTransform.basis * hitNormalLocal
                                val rayResult = CollisionWorld.LocalRayResult(collisionObject, shapeInfo, hitNormalWorld, hitFraction)
                                return resultCallback.addSingleResult(rayResult, normalInWorldSpace = true)
                            }
                        }

                        val rcb = BridgeTriangleRaycastCallback_(rayFromLocal, rayToLocal, resultCallback,
                                collisionObjectWrap.collisionObject!!, concaveShape, colObjWorldTransform)
                        rcb.hitFraction = resultCallback.closestHitFraction
                        val rayAabbMinLocal = rayFromLocal min rayToLocal
                        val rayAabbMaxLocal = rayFromLocal max rayToLocal
                        concaveShape.processAllTriangles(rcb, rayAabbMinLocal, rayAabbMaxLocal)
                    }
                }
            } else {
                BT_PROFILE("rayTestCompound")
                if (collisionShape.isCompound) {
                    class LocalInfoAdder2(val i: Int, val userCallback: RayResultCallback) : RayResultCallback() {
                        init {
                            closestHitFraction = userCallback.closestHitFraction
                            flags = userCallback.flags
                        }

                        override fun needsCollision(p: BroadphaseProxy) = userCallback.needsCollision(p)

                        override fun addSingleResult(rayResult: LocalRayResult, normalInWorldSpace: Boolean): Float {
                            val shapeInfo = CollisionWorld.LocalShapeInfo(shapePart = -1, triangleIndex = i)
                            if (rayResult.localShapeInfo == null) rayResult.localShapeInfo = shapeInfo
                            val result = userCallback.addSingleResult(rayResult, normalInWorldSpace)
                            closestHitFraction = userCallback.closestHitFraction
                            return result
                        }
                    }

                    class RayTester(val collisionObject: CollisionObject, val compoundShape: CompoundShape,
                                    val colObjWorldTransform: Transform, val rayFromTrans: Transform, val rayToTrans: Transform,
                                    val resultCallback: RayResultCallback) : Dbvt.Collide {

                        fun processLeaf(i: Int) {
                            val childCollisionShape = compoundShape.getChildShape(i)
                            val childTrans = compoundShape.getChildTransform(i)
                            val childWorldTrans = colObjWorldTransform * childTrans
                            val tmpOb = CollisionObjectWrapper(null, childCollisionShape, collisionObject, childWorldTrans, -1, i)
                            // replace collision shape so that callback can determine the triangle
                            val myCb = LocalInfoAdder2(i, resultCallback)
                            rayTestSingleInternal(rayFromTrans, rayToTrans, tmpOb, myCb)
                        }

                        override fun process(node: DbvtNode) = processLeaf(node.dataAsInt)
                    }

                    val compoundShape = collisionShape as CompoundShape
                    val dbvt = compoundShape.dynamicAabbTree

                    val rayCB = RayTester(collisionObjectWrap.collisionObject!!, compoundShape, colObjWorldTransform, rayFromTrans,
                            rayToTrans, resultCallback)
                    if (!DISABLE_DBVT_COMPOUNDSHAPE_RAYCAST_ACCELERATION && dbvt != null) {
                        val localRayFrom = colObjWorldTransform.inverseTimes(rayFromTrans).origin
                        val localRayTo = colObjWorldTransform.inverseTimes(rayToTrans).origin
                        Dbvt.rayTest(dbvt.root, localRayFrom, localRayTo, rayCB)
                    } else
                        for (i in 0 until compoundShape.numChildShapes) rayCB.processLeaf(i)
                }
            }
        }

        /** objectQuerySingle performs a collision detection query and calls the resultCallback. It is used internally
         *  by rayTest. */
        fun objectQuerySingle(castShape: ConvexShape, rayFromTrans: Transform, rayToTrans: Transform, collisionObject: CollisionObject,
                              collisionShape: CollisionShape, colObjWorldTransform: Transform, resultCallback: ConvexResultCallback,
                              allowedPenetration: Float) {

            val tmpOb = CollisionObjectWrapper(null, collisionShape, collisionObject, colObjWorldTransform, -1, -1)
            objectQuerySingleInternal(castShape, rayFromTrans, rayToTrans, tmpOb, resultCallback, allowedPenetration)
        }

        fun objectQuerySingleInternal(castShape: ConvexShape, convexFromTrans: Transform, convexToTrans: Transform,
                                      colObjWrap: CollisionObjectWrapper, resultCallback: ConvexResultCallback,
                                      allowedPenetration: Float) {
            val collisionShape = colObjWrap.collisionShape
            val colObjWorldTransform = colObjWrap.worldTransform

            if (collisionShape.isConvex) {
                BT_PROFILE("convexSweepConvex")
                val castResult = ConvexCast.CastResult().also { r ->
                    r.allowedPenetration = allowedPenetration
                    r.fraction = resultCallback.closestHitFraction//btScalar(1.);//??
                }
                val convexShape = collisionShape as ConvexShape
                val simplexSolver = VoronoiSimplexSolver()
                val gjkEpaPenetrationSolver = GjkEpaPenetrationDepthSolver()

                val convexCaster1 = ContinuousConvexCollision(castShape, convexShape, simplexSolver, gjkEpaPenetrationSolver)

                if (convexCaster1.calcTimeOfImpact(convexFromTrans, convexToTrans, colObjWorldTransform, colObjWorldTransform,
                                castResult)) {
                    //add hit
                    if (castResult.normal.length2() > 0.0001f) {
                        if (castResult.fraction < resultCallback.closestHitFraction) {
                            castResult.normal.normalize()
                            val localConvexResult = CollisionWorld.LocalConvexResult(colObjWrap.collisionObject!!, null,
                                    castResult.normal, castResult.hitPoint, castResult.fraction)
                            resultCallback.addSingleResult(localConvexResult, normalInWorldSpace = true)
                        }
                    }
                }
            } else {
                if (collisionShape.isConcave) {
                    if (collisionShape.shapeType == Bnt.TRIANGLE_MESH_SHAPE_PROXYTYPE) {
                        BT_PROFILE("convexSweepbtBvhTriangleMesh")
                        val triangleMesh = collisionShape as BvhTriangleMeshShape
                        val worldTocollisionObject = colObjWorldTransform.inverse()
                        val convexFromLocal = worldTocollisionObject * convexFromTrans.origin
                        val convexToLocal = worldTocollisionObject * convexToTrans.origin
                        // rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
                        val rotationXform = Transform(worldTocollisionObject.basis * convexToTrans.basis)

                        //ConvexCast::CastResult
                        class BridgeTriangleConvexcastCallback(castShape: ConvexShape, from: Transform, to: Transform,
                                                               val resultCallback: CollisionWorld.ConvexResultCallback,
                                                               val collisionObject: CollisionObject,
                                                               val triangleMesh: TriangleMeshShape,
                                                               triangleToWorld: Transform) :
                                TriangleConvexcastCallback(castShape, from, to, triangleToWorld, triangleMesh.margin) {

                            override fun reportHit(hitNormalLocal: Vec3, hitPointLocal: Vec3, hitFraction: Float, partId: Int,
                                                   triangleIndex: Int): Float {
                                val shapeInfo = CollisionWorld.LocalShapeInfo(partId, triangleIndex)
                                if (hitFraction <= resultCallback.closestHitFraction) {
                                    val convexResult = CollisionWorld.LocalConvexResult(collisionObject, shapeInfo, hitNormalLocal,
                                            hitPointLocal, hitFraction)
                                    return resultCallback.addSingleResult(convexResult, normalInWorldSpace = true)
                                }
                                return hitFraction
                            }
                        }

                        val tccb = BridgeTriangleConvexcastCallback(castShape, convexFromTrans, convexToTrans, resultCallback,
                                colObjWrap.collisionObject!!, triangleMesh, colObjWorldTransform)
                        tccb.hitFraction = resultCallback.closestHitFraction
                        tccb.allowedPenetration = allowedPenetration
                        val boxMinLocal = Vec3()
                        val boxMaxLocal = Vec3()
                        castShape.getAabb(rotationXform, boxMinLocal, boxMaxLocal)
                        triangleMesh.performConvexcast(tccb, convexFromLocal, convexToLocal, boxMinLocal, boxMaxLocal)
                    } else {
                        if (collisionShape.shapeType == Bnt.STATIC_PLANE_PROXYTYPE) {
                            val castResult = ConvexCast.CastResult()
                            castResult.allowedPenetration = allowedPenetration
                            castResult.fraction = resultCallback.closestHitFraction
                            val planeShape = collisionShape as StaticPlaneShape
                            val convexCaster1 = ContinuousConvexCollision(castShape, planeShape)
                            val castPtr = convexCaster1 as ConvexCast

                            if (castPtr.calcTimeOfImpact(convexFromTrans, convexToTrans, colObjWorldTransform, colObjWorldTransform,
                                            castResult) &&
                                    //add hit
                                    castResult.normal.length2() > 0.0001f && castResult.fraction < resultCallback.closestHitFraction) {
                                castResult.normal.normalize()
                                val localConvexResult = CollisionWorld.LocalConvexResult(colObjWrap.collisionObject!!,
                                        null, castResult.normal, castResult.hitPoint, castResult.fraction)
                                resultCallback.addSingleResult(localConvexResult, normalInWorldSpace = true)
                            }
                        } else {
                            BT_PROFILE("convexSweepConcave")
                            val concaveShape = collisionShape as ConcaveShape
                            val worldTocollisionObject = colObjWorldTransform.inverse()
                            val convexFromLocal = worldTocollisionObject * convexFromTrans.origin
                            val convexToLocal = worldTocollisionObject * convexToTrans.origin
                            // rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
                            val rotationXform = Transform(worldTocollisionObject.basis * convexToTrans.basis)

                            //ConvexCast::CastResult
                            class BridgeTriangleConvexcastCallback_(castShape: ConvexShape, from: Transform, to: Transform,
                                                                    val resultCallback: CollisionWorld.ConvexResultCallback,
                                                                    val collisionObject: CollisionObject,
                                                                    val triangleMesh: ConcaveShape, triangleToWorld: Transform)
                                : TriangleConvexcastCallback(castShape, from, to, triangleToWorld, triangleMesh.margin) {

                                override fun reportHit(hitNormalLocal: Vec3, hitPointLocal: Vec3, hitFraction: Float, partId: Int,
                                                       triangleIndex: Int): Float {
                                    val shapeInfo = CollisionWorld.LocalShapeInfo(partId, triangleIndex)
                                    if (hitFraction <= resultCallback.closestHitFraction) {
                                        val convexResult = CollisionWorld.LocalConvexResult(collisionObject, shapeInfo, hitNormalLocal,
                                                hitPointLocal, hitFraction)
                                        return resultCallback.addSingleResult(convexResult, normalInWorldSpace = true)
                                    }
                                    return hitFraction
                                }
                            }

                            val tccb = BridgeTriangleConvexcastCallback_(castShape, convexFromTrans, convexToTrans, resultCallback,
                                    colObjWrap.collisionObject!!, concaveShape, colObjWorldTransform)
                            tccb.hitFraction = resultCallback.closestHitFraction
                            tccb.allowedPenetration = allowedPenetration
                            val boxMinLocal = Vec3()
                            val boxMaxLocal = Vec3()
                            castShape.getAabb(rotationXform, boxMinLocal, boxMaxLocal)
                            val rayAabbMinLocal = convexFromLocal min convexToLocal
                            val rayAabbMaxLocal = convexFromLocal max convexToLocal
                            rayAabbMinLocal += boxMinLocal
                            rayAabbMaxLocal += boxMaxLocal
                            concaveShape.processAllTriangles(tccb, rayAabbMinLocal, rayAabbMaxLocal)
                        }
                    }
                } else {
                    if (collisionShape.isCompound) {
                        class CompoundLeafCallback(val colObjWrap: CollisionObjectWrapper, val castShape: ConvexShape,
                                                   val convexFromTrans: Transform, val convexToTrans: Transform,
                                                   val allowedPenetration: Float, val compoundShape: CompoundShape,
                                                   val colObjWorldTransform: Transform, val resultCallback: ConvexResultCallback)
                            : Dbvt.Collide {

                            fun processChild(index: Int, childTrans: Transform, childCollisionShape: CollisionShape) {
                                val childWorldTrans = colObjWorldTransform * childTrans

                                class LocalInfoAdder(val i: Int, val userCallback: ConvexResultCallback) : ConvexResultCallback() {

                                    init {
                                        closestHitFraction = userCallback.closestHitFraction
                                    }

                                    override fun needsCollision(p: BroadphaseProxy) = userCallback.needsCollision(p)
                                    override fun addSingleResult(convexResult: LocalConvexResult, normalInWorldSpace: Boolean): Float {
                                        val shapeInfo = CollisionWorld.LocalShapeInfo(-1, i)
                                        if (convexResult.localShapeInfo == null) convexResult.localShapeInfo = shapeInfo
                                        val result = userCallback.addSingleResult(convexResult, normalInWorldSpace)
                                        closestHitFraction = userCallback.closestHitFraction
                                        return result
                                    }
                                }

                                val myCb = LocalInfoAdder(index, resultCallback)
                                val tmpObj = CollisionObjectWrapper(colObjWrap, childCollisionShape, colObjWrap.collisionObject!!,
                                        childWorldTrans, -1, index)
                                objectQuerySingleInternal(castShape, convexFromTrans, convexToTrans, tmpObj, myCb, allowedPenetration)
                            }

                            override fun process(node: DbvtNode) {
                                // Processing leaf node
                                val index = node.dataAsInt
                                val childTrans = compoundShape.getChildTransform(index)
                                val childCollisionShape = compoundShape.getChildShape(index)
                                processChild(index, childTrans, childCollisionShape)
                            }
                        }
                        BT_PROFILE("convexSweepCompound")
                        val compoundShape = collisionShape as CompoundShape
                        val fromLocalAabbMin = Vec3()
                        val fromLocalAabbMax = Vec3()
                        val toLocalAabbMin = Vec3()
                        val toLocalAabbMax = Vec3()

                        castShape.getAabb(colObjWorldTransform.inverse() * convexFromTrans, fromLocalAabbMin, fromLocalAabbMax)
                        castShape.getAabb(colObjWorldTransform.inverse() * convexToTrans, toLocalAabbMin, toLocalAabbMax)

                        fromLocalAabbMin setMin toLocalAabbMin
                        fromLocalAabbMax setMax toLocalAabbMax

                        val callback = CompoundLeafCallback(colObjWrap, castShape, convexFromTrans, convexToTrans, allowedPenetration,
                                compoundShape, colObjWorldTransform, resultCallback)
                        val tree = compoundShape.dynamicAabbTree
                        if (tree != null)
                            tree.collideTV(tree.root, DbvtVolume.fromMM(fromLocalAabbMin, fromLocalAabbMax), callback)
                        else
                            for (i in 0 until compoundShape.numChildShapes) {
                                val childCollisionShape = compoundShape.getChildShape(i)
                                val childTrans = compoundShape.getChildTransform(i)
                                callback.processChild(i, childTrans, childCollisionShape)
                            }
                    }
                }
            }
        }
    }

    fun addCollisionObject(collisionObject: CollisionObject,
                           collisionFilterGroup: Int = BroadphaseProxy.CollisionFilterGroups.DefaultFilter.i,
                           collisionFilterMask: Int = BroadphaseProxy.CollisionFilterGroups.AllFilter.i) {
        //check that the object isn't already added
        assert(!collisionObjects.contains(collisionObject))
        assert(collisionObject.worldArrayIndex == -1)  // do not add the same object to more than one collision world

        collisionObject.worldArrayIndex = collisionObjects.size
        collisionObjects.add(collisionObject)
        //calculate new AABB
        val trans = collisionObject.getWorldTransform()

        val minAabb = Vec3()
        val maxAabb = Vec3()
        collisionObject.collisionShape!!.getAabb(trans, minAabb, maxAabb)

        val type = collisionObject.collisionShape!!.shapeType
        collisionObject.broadphaseHandle = broadphase.createProxy(minAabb, maxAabb, type.i, collisionObject, collisionFilterGroup,
                collisionFilterMask, dispatcher!!)
    }

    open fun removeCollisionObject(collisionObject: CollisionObject) {
        collisionObject.broadphaseHandle?.let {
            //
            // only clear the cached algorithms
            //
            broadphase.overlappingPairCache.cleanProxyFromPairs(it, dispatcher!!)
            broadphase.destroyProxy(it, dispatcher!!)
            collisionObject.broadphaseHandle = null
        }
        val iObj = collisionObject.worldArrayIndex
//    btAssert(iObj >= 0 && iObj < m_collisionObjects.size()); // trying to remove an object that was never added or already removed previously?
        if (iObj in collisionObjects.indices) {
            assert(collisionObject === collisionObjects[iObj])
            collisionObjects.swapLastAt(iObj)
            collisionObjects.pop()
            if (iObj < collisionObjects.size)
                collisionObjects[iObj].worldArrayIndex = iObj
        } else
        // slow linear search
        //swapremove
            collisionObjects.remove(collisionObject)

        collisionObject.worldArrayIndex = -1
    }

    fun performDiscreteCollisionDetection() {
        BT_PROFILE("performDiscreteCollisionDetection")
        updateAabbs()
        computeOverlappingPairs()
        BT_PROFILE("dispatchAllCollisionPairs")
        dispatcher?.let {
            it.dispatchAllCollisionPairs(broadphasePairCache.overlappingPairCache, dispatchInfo, it)
        }
    }
}

class SingleRayCallback(val rayFromWorld: Vec3, val rayToWorld: Vec3, val world: CollisionWorld, val resultCallback: CollisionWorld.RayResultCallback)
    : BroadphaseRayCallback() {

    val rayFromTrans = Transform().apply { setIdentity(); origin put rayFromWorld }
    val rayToTrans = Transform().apply { setIdentity(); origin put rayToWorld }
    val hitNormal = Vec3()

    init {
        val rayDir = rayToWorld - rayFromWorld

        rayDir.normalize()
        // what about division by zero? --> just set rayDirection[i] to INF/BT_LARGE_FLOAT
        rayDirectionInverse[0] = if (rayDir[0] == 0f) LARGE_FLOAT else 1f / rayDir[0]
        rayDirectionInverse[1] = if (rayDir[1] == 0f) LARGE_FLOAT else 1f / rayDir[1]
        rayDirectionInverse[2] = if (rayDir[2] == 0f) LARGE_FLOAT else 1f / rayDir[2]
        signs[0] = (rayDirectionInverse[0] < 0f).i
        signs[1] = (rayDirectionInverse[1] < 0f).i
        signs[2] = (rayDirectionInverse[2] < 0f).i

        lambdaMax = rayDir dot (rayToWorld - rayFromWorld)
    }

    override fun process(proxy: BroadphaseProxy): Boolean {
        ///terminate further ray tests, once the closestHitFraction reached zero
        if (resultCallback.closestHitFraction == 0f) return false

        val collisionObject = proxy.clientObject as CollisionObject

        // only perform raycast if filterMask matches
        if (resultCallback needsCollision collisionObject.broadphaseHandle!!)
        //culling already done by broadphase
            CollisionWorld.rayTestSingle(rayFromTrans, rayToTrans, collisionObject, collisionObject.collisionShape!!,
                    collisionObject.getWorldTransform(), resultCallback)
        return true
    }
}

class SingleSweepCallback(val castShape: ConvexShape, val convexFromTrans: Transform, val convexToTrans: Transform, val world: CollisionWorld,
                          val resultCallback: CollisionWorld.ConvexResultCallback, val allowedCcdPenetration: Float) : BroadphaseRayCallback() {

    val hitNormal = Vec3()

    init {
        val unnormalizedRayDir = convexToTrans.origin - convexFromTrans.origin
        val rayDir = unnormalizedRayDir.normalized()
        ///what about division by zero? --> just set rayDirection[i] to INF/BT_LARGE_FLOAT
        rayDirectionInverse[0] = if (rayDir[0] == 0f) LARGE_FLOAT else 1f / rayDir[0]
        rayDirectionInverse[1] = if (rayDir[1] == 0f) LARGE_FLOAT else 1f / rayDir[1]
        rayDirectionInverse[2] = if (rayDir[2] == 0f) LARGE_FLOAT else 1f / rayDir[2]
        signs[0] = (rayDirectionInverse[0] < 0f).i
        signs[1] = (rayDirectionInverse[1] < 0f).i
        signs[2] = (rayDirectionInverse[2] < 0f).i

        lambdaMax = rayDir dot unnormalizedRayDir
    }

    override fun process(proxy: BroadphaseProxy): Boolean {
        ///terminate further convex sweep tests, once the closestHitFraction reached zero
        if (resultCallback.closestHitFraction == 0f) return false

        val collisionObject = proxy.clientObject as CollisionObject

        // only perform raycast if filterMask matches
        if (resultCallback needsCollision collisionObject.broadphaseHandle!!)
            CollisionWorld.objectQuerySingle(castShape, convexFromTrans, convexToTrans, collisionObject,
                    collisionObject.collisionShape!!, collisionObject.getWorldTransform(), resultCallback, allowedCcdPenetration)
        return true
    }
}

class SingleContactCallback(val collisionObject: CollisionObject, val world: CollisionWorld, val resultCallback: CollisionWorld.ContactResultCallback) :
        BroadphaseAabbCallback {

    override fun process(proxy: BroadphaseProxy): Boolean {

        val collisionObject = proxy.clientObject as CollisionObject
        if (collisionObject == collisionObject) return true

        // only perform raycast if filterMask matches
        if (resultCallback needsCollision collisionObject.broadphaseHandle!!) {
            val ob0 = CollisionObjectWrapper(null, collisionObject.collisionShape, collisionObject, collisionObject.getWorldTransform(), -1, -1)
            val ob1 = CollisionObjectWrapper(null, collisionObject.collisionShape, collisionObject, collisionObject.getWorldTransform(), -1, -1)

            world.dispatcher!!.findAlgorithm(ob0, ob1, null, Dqt.CLOSEST_POINT_ALGORITHMS)?.let { algorithm ->
                val contactPointResult = BridgedManifoldResult(ob0, ob1, resultCallback)
                //discrete collision detection query

                algorithm.processCollision(ob0, ob1, world.dispatchInfo, contactPointResult)

                world.dispatcher!!.freeCollisionAlgorithm(algorithm)
            }
        }
        return true
    }
}

class BridgedManifoldResult(obj0Wrap: CollisionObjectWrapper, obj1Wrap: CollisionObjectWrapper, val resultCallback: CollisionWorld.ContactResultCallback)
    : ManifoldResult(obj0Wrap, obj1Wrap) {

    override fun addContactPoint(normalOnBInWorld: Vec3, pointInWorld: Vec3, depth: Float) {

        val isSwapped = manifold!!.body0 !== body0Wrap!!.collisionObject
        val pointA = pointInWorld + normalOnBInWorld * depth
        val localA = Vec3()
        val localB = Vec3()
        if (isSwapped) {
            localA put body1Wrap!!.collisionObject!!.getWorldTransform().invXform(pointA)
            localB put body0Wrap!!.collisionObject!!.getWorldTransform().invXform(pointInWorld)
        } else {
            localA put body0Wrap!!.collisionObject!!.getWorldTransform().invXform(pointA)
            localB put body1Wrap!!.collisionObject!!.getWorldTransform().invXform(pointInWorld)
        }

        val newPt = ManifoldPoint(localA, localB, normalOnBInWorld, depth)
        newPt.positionWorldOnA put pointA
        newPt.positionWorldOnB put pointInWorld

        //BP mod, store contact triangles.
        if (isSwapped) {
            newPt.partId0 = partId1
            newPt.partId1 = partId0
            newPt.index0 = index1
            newPt.index1 = index0
        } else {
            newPt.partId0 = partId0
            newPt.partId1 = partId1
            newPt.index0 = index0
            newPt.index1 = index1
        }

        // experimental feature info, for per-triangle material etc.
        val obj0Wrap = if (isSwapped) body1Wrap else body0Wrap
        val obj1Wrap = if (isSwapped) body0Wrap else body1Wrap
        resultCallback.addSingleResult(newPt, obj0Wrap, newPt.partId0, newPt.index0, obj1Wrap, newPt.partId1, newPt.index1)

    }

}