package bullet

import bullet.linearMath.Transform
import bullet.linearMath.Vec3

interface ConvexTemplate {

    var worldTrans: Transform

    var margin: Float

    fun getLocalSupportWithMargin(dir: Vec3): Vec3
    fun getLocalSupportWithoutMargin(dir: Vec3): Vec3

    val objectCenterInWorld: Vec3
}

abstract class DistanceTemplate {

    var pointOnA = Vec3()
    var pointOnB = Vec3()
    var normalBtoA = Vec3()
    var distance = 0f
}