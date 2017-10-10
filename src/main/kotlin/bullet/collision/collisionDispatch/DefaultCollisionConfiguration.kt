package bullet.collision.collisionDispatch

import bullet.collision.narrowPhaseCollision.ConvexPenetrationDepthSolver
import bullet.collision.narrowPhaseCollision.GjkEpaPenetrationDepthSolver

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
) :
        CollisionConfiguration() {

    private var persistentManifoldPoolSize = 0


    //    btPoolAllocator*	m_persistentManifoldPool;
    private var ownsPersistentManifoldPool = false


    //    btPoolAllocator*	m_collisionAlgorithmPool;
    private var ownsCollisionAlgorithmPool = false

    override fun getCollisionAlgorithmCreateFunc(proxyType0: Int, proxyType1: Int): CollisionAlgorithmCreateFunc {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getClosestPointsAlgorithmCreateFunc(proxyType0:Int, proxyType1:Int) : CollisionAlgorithmCreateFunc {
        TODO()
    }
//
//    ///Use this method to allow to generate multiple contact points between at once, between two objects using the generic convex-convex algorithm.
//    ///By default, this feature is disabled for best performance.
//    ///@param numPerturbationIterations controls the number of collision queries. Set it to zero to disable the feature.
//    ///@param minimumPointsPerturbationThreshold is the minimum number of points in the contact cache, above which the feature is disabled
//    ///3 is a good value for both params, if you want to enable the feature. This is because the default contact cache contains a maximum of 4 points, and one collision query at the unperturbed orientation is performed first.
//    ///See Bullet/Demos/CollisionDemo for an example how this feature gathers multiple points.
//    ///@todo we could add a per-object setting of those parameters, for level-of-detail collision detection.
//    void    setConvexConvexMultipointIterations(int numPerturbationIterations=3, int minimumPointsPerturbationThreshold = 3);
//
//    void    setPlaneConvexMultipointIterations(int numPerturbationIterations=3, int minimumPointsPerturbationThreshold = 3);
}