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

package bullet.collision.collisionDispatch

import bullet.collision.broadphaseCollision.isCompound
import bullet.collision.broadphaseCollision.isConcave
import bullet.collision.broadphaseCollision.isConvex
import bullet.collision.narrowPhaseCollision.GjkEpaPenetrationDepthSolver
import bullet.collision.narrowPhaseCollision.MinkowskiPenetrationDepthSolver
import bullet.collision.broadphaseCollision.BroadphaseNativeTypes as Bnt

class DefaultCollisionConstructionInfo {
    // TODO    btPoolAllocator*	m_persistentManifoldPool;
//    btPoolAllocator*	m_collisionAlgorithmPool;
    var defaultMaxPersistentManifoldPoolSize = 4096
    var defaultMaxCollisionAlgorithmPoolSize = 4096
    val customCollisionAlgorithmMaxElementSize = 0
    var useEpaPenetrationAlgorithm = true
}

/** CollisionConfiguration allows to configure Bullet collision detection, stack allocator, pool memory allocators
 *  todo: describe the meaning  */
class DefaultCollisionConfiguration(
        constructionInfo: DefaultCollisionConstructionInfo = DefaultCollisionConstructionInfo()
) : CollisionConfiguration() {

    private var persistentManifoldPoolSize = 0

    //    btPoolAllocator*	m_persistentManifoldPool;
    private var ownsPersistentManifoldPool = false

    //    btPoolAllocator*	m_collisionAlgorithmPool;
    private var ownsCollisionAlgorithmPool = false

    /** default penetration depth solver    */
    val solver = if (constructionInfo.useEpaPenetrationAlgorithm) GjkEpaPenetrationDepthSolver() else MinkowskiPenetrationDepthSolver()

    //default CreationFunctions, filling the m_doubleDispatch table
    val convexConvexCreateFunc = ConvexConvexAlgorithm.CreateFunc(solver)
    val convexConcaveCreateFunc = ConvexConcaveCollisionAlgorithm.CreateFunc()
    val swappedConvexConcaveCreateFunc = ConvexConcaveCollisionAlgorithm.SwappedCreateFunc()
    val compoundCreateFunc = CompoundCollisionAlgorithm.CreateFunc()
    val compoundCompoundCreateFunc = CompoundCompoundCollisionAlgorithm.CreateFunc()

    val swappedCompoundCreateFunc = CompoundCollisionAlgorithm.SwappedCreateFunc()
    val emptyCreateFunc = EmptyAlgorithm.CreateFunc()
    val sphereSphereCF = SphereSphereCollisionAlgorithm.CreateFunc()

    val boxBoxCF = BoxBoxCollisionAlgorithm.CreateFunc()
    val sphereTriangleCF = SphereTriangleCollisionAlgorithm.CreateFunc()
    val triangleSphereCF = SphereTriangleCollisionAlgorithm.CreateFunc()
    val planeConvexCF = ConvexPlaneCollisionAlgorithm.CreateFunc()
    val convexPlaneCF = ConvexPlaneCollisionAlgorithm.CreateFunc()

    override fun getCollisionAlgorithmCreateFunc(proxyType0: Bnt, proxyType1: Bnt) = when {
        proxyType0 == Bnt.SPHERE_SHAPE_PROXYTYPE && proxyType1 == Bnt.SPHERE_SHAPE_PROXYTYPE -> sphereSphereCF
        proxyType0 == Bnt.SPHERE_SHAPE_PROXYTYPE && proxyType1 == Bnt.TRIANGLE_SHAPE_PROXYTYPE -> sphereTriangleCF
        proxyType0 == Bnt.TRIANGLE_SHAPE_PROXYTYPE && proxyType1 == Bnt.SPHERE_SHAPE_PROXYTYPE -> triangleSphereCF
        proxyType0 == Bnt.BOX_SHAPE_PROXYTYPE && proxyType1 == Bnt.BOX_SHAPE_PROXYTYPE -> boxBoxCF
        proxyType0.isConvex && proxyType1 == Bnt.STATIC_PLANE_PROXYTYPE -> convexPlaneCF
        proxyType1.isConvex && proxyType0 == Bnt.STATIC_PLANE_PROXYTYPE -> planeConvexCF
        proxyType0.isConvex && proxyType1.isConvex -> convexConvexCreateFunc
        proxyType0.isConvex && proxyType1.isConcave -> convexConcaveCreateFunc
        proxyType1.isConvex && proxyType0.isConcave -> swappedConvexConcaveCreateFunc
        proxyType0.isCompound && proxyType1.isCompound -> compoundCompoundCreateFunc
        proxyType0.isCompound -> compoundCreateFunc
        proxyType1.isCompound -> swappedCompoundCreateFunc
        else -> emptyCreateFunc // failed to find an algorithm
    }

    override fun getClosestPointsAlgorithmCreateFunc(proxyType0: Bnt, proxyType1: Bnt) = when {
        proxyType0 == Bnt.SPHERE_SHAPE_PROXYTYPE && proxyType1 == Bnt.SPHERE_SHAPE_PROXYTYPE -> sphereSphereCF
        proxyType0 == Bnt.SPHERE_SHAPE_PROXYTYPE && proxyType1 == Bnt.TRIANGLE_SHAPE_PROXYTYPE -> sphereTriangleCF
        proxyType0 == Bnt.TRIANGLE_SHAPE_PROXYTYPE && proxyType1 == Bnt.SPHERE_SHAPE_PROXYTYPE -> triangleSphereCF
        proxyType0.isConvex && proxyType1 == Bnt.STATIC_PLANE_PROXYTYPE -> convexPlaneCF
        proxyType1.isConvex && proxyType0 == Bnt.STATIC_PLANE_PROXYTYPE -> planeConvexCF
        proxyType0.isConvex && proxyType1.isConvex -> convexConvexCreateFunc
        proxyType0.isConvex && proxyType1.isConcave -> convexConcaveCreateFunc
        proxyType1.isConvex && proxyType0.isConcave -> swappedConvexConcaveCreateFunc
        proxyType0.isCompound && proxyType1.isCompound -> compoundCompoundCreateFunc
        proxyType0.isCompound -> compoundCreateFunc
        proxyType1.isCompound -> swappedCompoundCreateFunc
        else -> emptyCreateFunc // failed to find an algorithm
    }

    /** Use this method to allow to generate multiple contact points between at once, between two objects using
     *  the generic convex-convex algorithm.
     *  By default, this feature is disabled for best performance.
     *  @param numPerturbationIterations controls the number of collision queries. Set it to zero to disable the feature
     *  @param minimumPointsPerturbationThreshold is the minimum number of points in the contact cache, above which
     *      the feature is disabled
     *  3 is a good value for both params, if you want to enable the feature. This is because the default contact cache
     *  contains a maximum of 4 points, and one collision query at the unperturbed orientation is performed first.
     *  See Bullet/Demos/CollisionDemo for an example how this feature gathers multiple points.
     *  @todo we could add a per-object setting of those parameters, for level-of-detail collision detection.   */
    fun setConvexConvexMultipointIterations(numPerturbationIterations: Int = 3, minimumPointsPerturbationThreshold: Int = 3) {
        convexConvexCreateFunc.numPerturbationIterations = numPerturbationIterations
        convexConvexCreateFunc.minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold
    }

    fun setPlaneConvexMultipointIterations(numPerturbationIterations: Int = 3, minimumPointsPerturbationThreshold: Int = 3) {

        convexPlaneCF.numPerturbationIterations = numPerturbationIterations
        convexPlaneCF.minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold

        planeConvexCF.numPerturbationIterations = numPerturbationIterations
        planeConvexCF.minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold
    }
}