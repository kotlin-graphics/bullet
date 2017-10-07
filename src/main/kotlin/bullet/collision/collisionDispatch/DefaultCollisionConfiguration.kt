package bullet.collision.collisionDispatch

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

    /** default penetration depth solver    */
    btConvexPenetrationDepthSolver*	m_pdSolver;

    //default CreationFunctions, filling the m_doubleDispatch table
    private var convexConvexCreateFunc: CollisionAlgorithmCreateFunc
    private lateinit var convexConcaveCreateFunc: CollisionAlgorithmCreateFunc
    private lateinit var swappedConvexConcaveCreateFunc: CollisionAlgorithmCreateFunc
    private lateinit var compoundCreateFunc: CollisionAlgorithmCreateFunc
    private lateinit var compoundCompoundCreateFunc: CollisionAlgorithmCreateFunc

    private lateinit var swappedCompoundCreateFunc: CollisionAlgorithmCreateFunc
    private lateinit var emptyCreateFunc: CollisionAlgorithmCreateFunc
    private lateinit var sphereSphereCF: CollisionAlgorithmCreateFunc
    private lateinit var sphereBoxCF: CollisionAlgorithmCreateFunc
    private lateinit var boxSphereCF: CollisionAlgorithmCreateFunc

    private lateinit var boxBoxCF: CollisionAlgorithmCreateFunc
    private lateinit var sphereTriangleCF: CollisionAlgorithmCreateFunc
    private lateinit var triangleSphereCF: CollisionAlgorithmCreateFunc
    private lateinit var planeConvexCF: CollisionAlgorithmCreateFunc
    private lateinit var convexPlaneCF: CollisionAlgorithmCreateFunc

    // memory pools TODO
//    virtual btPoolAllocator* getPersistentManifoldPool()
//    {
//        return m_persistentManifoldPool;
//    }
//
//    virtual btPoolAllocator* getCollisionAlgorithmPool()
//    {
//        return m_collisionAlgorithmPool;
//    }


    virtual btCollisionAlgorithmCreateFunc* getCollisionAlgorithmCreateFunc(int proxyType0,int proxyType1);

    virtual btCollisionAlgorithmCreateFunc* getClosestPointsAlgorithmCreateFunc(int proxyType0, int proxyType1);

    ///Use this method to allow to generate multiple contact points between at once, between two objects using the generic convex-convex algorithm.
    ///By default, this feature is disabled for best performance.
    ///@param numPerturbationIterations controls the number of collision queries. Set it to zero to disable the feature.
    ///@param minimumPointsPerturbationThreshold is the minimum number of points in the contact cache, above which the feature is disabled
    ///3 is a good value for both params, if you want to enable the feature. This is because the default contact cache contains a maximum of 4 points, and one collision query at the unperturbed orientation is performed first.
    ///See Bullet/Demos/CollisionDemo for an example how this feature gathers multiple points.
    ///@todo we could add a per-object setting of those parameters, for level-of-detail collision detection.
    void    setConvexConvexMultipointIterations(int numPerturbationIterations=3, int minimumPointsPerturbationThreshold = 3);

    void    setPlaneConvexMultipointIterations(int numPerturbationIterations=3, int minimumPointsPerturbationThreshold = 3);

};