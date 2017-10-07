package bullet.collision.collisionShapes

import bullet.collision.broadphaseCollision.BroadphaseNativeTypes
import bullet.linearMath.Transform
import bullet.linearMath.Vec3

/** The CollisionShape class provides an interface for collision shapes that can be shared among CollisionObjects.  */
open class CollisionShape {

    var shapeType = BroadphaseNativeTypes.INVALID_SHAPE_PROXYTYPE
//    void* m_userPointer;
    var userIndex = -1

    fun getBoundingSphere(center:Vec3,radius:Float)     {
        val tr = Transform()
        tr.setIdentity();
        btVector3 aabbMin,aabbMax;

        getAabb(tr,aabbMin,aabbMax);

        radius = (aabbMax-aabbMin).length()*btScalar(0.5);
        center = (aabbMin+aabbMax)*btScalar(0.5);
    }
}