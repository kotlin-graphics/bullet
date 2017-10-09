package bullet.collision.narrowPhaseCollision

import bullet.linearMath.Transform
import bullet.linearMath.Vec3

interface ConvexInterface {

    var worldTrans: Transform

    fun getLocalSupportWithMargin(dir: Vec3): Vec3
}