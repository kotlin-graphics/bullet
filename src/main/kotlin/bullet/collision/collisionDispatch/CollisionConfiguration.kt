package bullet.collision.collisionDispatch

/** CollisionConfiguration allows to configure Bullet collision detection
 *  stack allocator size, default collision algorithms and persistent manifold pool size
 *  todo: describe the meaning  */
abstract class CollisionConfiguration {

    ///memory pools TODO
//    virtual btPoolAllocator* getPersistentManifoldPool() = 0;
//
//    virtual btPoolAllocator* getCollisionAlgorithmPool() = 0;

    abstract fun getCollisionAlgorithmCreateFunc(proxyType0: Int,proxyType1:Int): CollisionAlgorithmCreateFunc

    abstract fun getClosestPointsAlgorithmCreateFunc(proxyType0:Int, proxyType1:Int): CollisionAlgorithmCreateFunc

}