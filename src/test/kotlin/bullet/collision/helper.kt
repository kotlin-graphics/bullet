package bullet.collision

import bullet.ConvexTemplate
import bullet.DistanceTemplate
import bullet.EPSILON
import bullet.collision.collisionShapes.ConvexShape
import bullet.collision.collisionShapes.MultiSphereShape
import bullet.collision.collisionShapes.SphereShape
import bullet.collision.narrowPhaseCollision.*
import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import bullet.linearMath.times
import bullet.collision.Collision.SphereSphereTestMethod as SSTM

class SphereSphereCollisionDescription {

    val sphereTransformA = Transform()
    val sphereTransformB = Transform()
    var radiusA = 0f
    var radiusB = 0f
}

class DistanceInfo : DistanceTemplate()

/** compute the distance between two spheres, where the distance is zero when the spheres are touching
 *  positive distance means the spheres are separate and negative distance means penetration
 *  point A and pointB are witness points, and normalOnB points from sphere B to sphere A   */
fun computeSphereSphereCollision(input: SphereSphereCollisionDescription, distInfo: DistanceInfo): Int {

    val diff = input.sphereTransformA.origin - input.sphereTransformB.origin
    val len = diff.length()
    val radiusA = input.radiusA
    val radiusB = input.radiusB

    ///distance (negative means penetration)
    val dist = len - (radiusA + radiusB)
    var normalOnSurfaceB = Vec3(1f, 0f, 0f)
    if (len > Float.EPSILON)
        normalOnSurfaceB = diff / len
    with(distInfo) {
        distance = dist
        normalBtoA = normalOnSurfaceB
        pointOnA = input.sphereTransformA.origin - input.radiusA * normalOnSurfaceB
        pointOnB = input.sphereTransformB.origin + input.radiusB * normalOnSurfaceB
    }
    return 0//sphere-sphere cannot fail
}

fun computeGjkEpaSphereSphereCollision(input: SphereSphereCollisionDescription, distInfo: DistanceInfo,
                                       method: Collision.SphereSphereTestMethod): Int {
    //  for spheres it is best to use a 'point' and set the margin to the radius (which is what btSphereShape does)
    val singleSphereA = SphereShape(input.radiusA)
    val singleSphereB = SphereShape(input.radiusB)
    val org = Vec3()
    val radA = input.radiusA
    val radB = input.radiusB

    val a = ConvexWrap()
    val b = ConvexWrap()
    a.worldTrans = input.sphereTransformA
    b.worldTrans = input.sphereTransformB

    val multiSphereA = MultiSphereShape(org, radA, 1)
    val multiSphereB = MultiSphereShape(org, radB, 1)

    val colDesc = GjkCollisionDescription()
    when (method) {
        SSTM.GJKEPA_RADIUS_NOT_FULL_MARGIN -> {
            a.convex = multiSphereA
            b.convex = multiSphereB
        }
        else -> {
            a.convex = singleSphereA
            b.convex = singleSphereB
        }
    }

    val simplexSolver = VoronoiSimplexSolver()
    simplexSolver.reset()

    var res = -1
    ///todo(erwincoumans): improve convex-convex quality and performance
    ///also compare with https://code.google.com/p/bullet/source/browse/branches/PhysicsEffects/src/base_level/collision/pfx_gjk_solver.cpp
    when (method) {
        SSTM.GJKEPA_RADIUS_NOT_FULL_MARGIN, SSTM.GJKEPA -> res = computeGjkEpaPenetration(a, b, colDesc, simplexSolver, distInfo)
        SSTM.GJKMPR -> {
            res = computeGjkDistance(a, b, colDesc, distInfo)
            if (res == 0)
            //   printf("use GJK results in distance %f\n",distInfo->m_distance);
                return res
            else
                res = Mpr.computeMprPenetration(a, b, Mpr.CollisionDescription(), distInfo)
        }
        else -> throw Error()
    }
    return res
}

class ConvexWrap : ConvexTemplate {

    lateinit var convex: ConvexShape
    override lateinit var worldTrans: Transform

    override var margin = 0f
        get() = convex.margin
    override val objectCenterInWorld get() = worldTrans.origin
    override fun getLocalSupportWithMargin(dir: Vec3) = convex.localGetSupportingVertex(dir)
    override fun getLocalSupportWithoutMargin(dir: Vec3) = convex.localGetSupportingVertexWithoutMargin(dir)
}