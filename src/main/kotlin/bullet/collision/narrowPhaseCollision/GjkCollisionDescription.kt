package bullet.collision.narrowPhaseCollision

import bullet.linearMath.Vec3

class GjkCollisionDescription {

    var firstDir = Vec3(0f, 1f, 0f)
    var maxGjkIterations = 1000
    var maximumDistanceSquared = 1e30f
    var gjkRelError2 = 1e-6f
}