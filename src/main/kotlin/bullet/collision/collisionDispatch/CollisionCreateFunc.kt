package bullet.collision.collisionDispatch

/** Used by the btCollisionDispatcher to register and create instances for btCollisionAlgorithm */
abstract class CollisionAlgorithmCreateFunc {

    var swapped = false

//    TODO abstract createCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& , const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap)
}