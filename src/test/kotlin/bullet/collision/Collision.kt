package bullet.collision

import bullet.f
import bullet.linearMath.Vec3
import bullet.linearMath.times
import io.kotlintest.matchers.shouldBe
import io.kotlintest.specs.StringSpec
import kotlin.math.abs
import bullet.collision.Collision.SphereSphereTestMethod as SSTM

class Collision : StringSpec() {

    enum class SphereSphereTestMethod { ANALYTIC, GJKEPA, GJKEPA_RADIUS_NOT_FULL_MARGIN, GJKMPR }

    init {

//        "test sphere sphere distance, (SSTM.GJKMPR, 0.0001f)" {
//            testSphereSphereDistance(SSTM.GJKMPR, 0.0001f)
//        }
//        "test sphere sphere distance, (SSTM.GJKEPA, 0.00001f)" {
//            testSphereSphereDistance(SSTM.GJKEPA, 0.00001f)
//        }
        "test sphere sphere distance, (SSTM.GJKEPA_RADIUS_NOT_FULL_MARGIN, 0.1f)" {
            testSphereSphereDistance(SSTM.GJKEPA_RADIUS_NOT_FULL_MARGIN, 0.1f)
        }
    }

    fun testSphereSphereDistance(method: SSTM, absError: Float) {

        run {
            val ssd = SphereSphereCollisionDescription()
            ssd.sphereTransformA.setIdentity()
            ssd.sphereTransformB.setIdentity()
            val distInfo = DistanceInfo()
            val result = computeSphereSphereCollision(ssd, distInfo)
            result shouldBe 0
            distInfo.distance shouldBe 0f
        }

        for (rb in 1..4)
            for (z in -5..4)
                for (j in 1..4)
                    for (i in -5..4)
                        if (i != z) {   //skip co-centric spheres for now (todo(erwincoumans) fix this)
                            val ssd = SphereSphereCollisionDescription().apply {
                                sphereTransformA.setIdentity()
                                sphereTransformA.origin = Vec3(0f, i.f, 0f)
                                sphereTransformB.setIdentity()
                                sphereTransformB.origin = Vec3(0f, z.f, 0f)
                                radiusA = j.f
                                radiusB = rb.f * 0.1f
                            }
                            val distInfo = DistanceInfo()
                            val result = when (method) {
                                SSTM.ANALYTIC -> computeSphereSphereCollision(ssd, distInfo)
                                else -> computeGjkEpaSphereSphereCollision(ssd, distInfo, method)
                            }
                            //  int result = btComputeSphereSphereCollision(ssd,&distInfo);
                            result shouldBe 0
                            val distance = abs((i - z).f) - j.f - ssd.radiusB
                            assert(distInfo.distance in distance - absError..distance + absError) // TODO plusOrMinus kotlinTest
                            val computedA = distInfo.pointOnB + distInfo.distance * distInfo.normalBtoA
                            val p = distInfo.pointOnA
                            assert(computedA.x in p.x - absError..p.x + absError)
                            assert(computedA.y in p.y - absError..p.y + absError)
                            assert(computedA.z in p.z - absError..p.z + absError)
                        }
    }
}