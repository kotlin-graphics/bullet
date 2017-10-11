package bullet.collision.collisionDispatch

import bullet.collision.collisionShapes.CollisionShape
import bullet.linearMath.Transform

abstract class CollisionObjectWrapper {

    /** not implemented. Not allowed.   */
    private constructor(collisionObjectWrapper: CollisionObjectWrapper)

    abstract infix fun put(collisionObjectWrapper: CollisionObjectWrapper): CollisionObjectWrapper

    var parent: CollisionObjectWrapper? =null
    var shape:CollisionShape?=null
    var collisionObject:CollisionObject?=null
    var worldTransform = Transform()
    var partId = 0
    var index =0

    // TODO check constructors
    constructor(parent: CollisionObjectWrapper, shape: CollisionShape, collisionObject: CollisionObject, worldTransform: Transform,
                partId: Int, index: Int) {
        this.parent = parent
        this.shape = shape
        this.collisionObject = collisionObject
        this.worldTransform = worldTransform
        this.partId = partId
        this.index = index
    }
}