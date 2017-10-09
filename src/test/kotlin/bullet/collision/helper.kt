package bullet.collision

import bullet.EPSILON
import bullet.collision.collisionShapes.SphereShape
import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import bullet.linearMath.times

class SphereSphereCollisionDescription {

    val sphereTransformA = Transform()
    val sphereTransformB = Transform()
    var radiusA = 0f
    var radiusB = 0f
}

class DistanceInfo {
    var pointOnA = Vec3()
    var pointOnB = Vec3()
    var normalBtoA = Vec3()
    var distance = 0f
}

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
    btSphereShape singleSphereB (input.m_radiusB);
    btVector3 org (0, 0, 0);
    btScalar radA = input . m_radiusA;
    btScalar radB = input . m_radiusB;

    ConvexWrap a, b;
    a.m_worldTrans = input.m_sphereTransformA;
    b.m_worldTrans = input.m_sphereTransformB;;

    btMultiSphereShape multiSphereA (&org, &radA, 1);
    btMultiSphereShape multiSphereB (&org, &radB, 1);

    btGjkCollisionDescription colDesc;
    switch(method)
    {
        case SSTM_GJKEPA_RADIUS_NOT_FULL_MARGIN :
        {

            a.m_convex = & multiSphereA;
            b.m_convex = & multiSphereB;
            break;
        }
        default:
        {
            a.m_convex = & singleSphereA;
            b.m_convex = & singleSphereB;
        }
    };

    btVoronoiSimplexSolver simplexSolver;
    simplexSolver.reset();

    int res = - 1;
    ///todo(erwincoumans): improve convex-convex quality and performance
    ///also compare with https://code.google.com/p/bullet/source/browse/branches/PhysicsEffects/src/base_level/collision/pfx_gjk_solver.cpp
    switch(method)
    {
        case SSTM_GJKEPA_RADIUS_NOT_FULL_MARGIN :
        case SSTM_GJKEPA :
        {
            res = btComputeGjkEpaPenetration(a, b, colDesc, simplexSolver, distInfo);
            break;
        }
        case SSTM_GJKMPR :
        {
            res = btComputeGjkDistance(a, b, colDesc, distInfo);
            if (res == 0) {
                //   printf("use GJK results in distance %f\n",distInfo->m_distance);
                return res;
            } else {
                btMprCollisionDescription mprDesc;
                res = btComputeMprPenetration(a, b, mprDesc, distInfo);

//                if (res==0)
//                {
//                    printf("use MPR results in distance %f\n",distInfo->m_distance);
//                }
            }
            break;
        }
        default:
        {

            btAssert(0);
        }
    }
    return res;
}