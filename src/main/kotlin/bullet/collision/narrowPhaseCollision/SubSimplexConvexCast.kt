/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

package bullet.collision.narrowPhaseCollision

import bullet.EPSILON
import bullet.collision.collisionShapes.ConvexShape
import bullet.linearMath.Transform
import bullet.linearMath.Vec3
import bullet.linearMath.times

private val MAX_ITERATIONS = 32

/** SubsimplexConvexCast implements Gino van den Bergens' paper
 *  "Ray Casting against bteral Convex Objects with Application to Continuous Collision Detection"
 *  GJK based Ray Cast, optimized version
 *  Objects should not start in overlap, otherwise results are not defined. */
class SubsimplexConvexCast(val convexA: ConvexShape, val convexB: ConvexShape, val simplexSolver: SimplexSolverInterface)
    : ConvexCast() {

    /** SimsimplexConvexCast calculateTimeOfImpact calculates the time of impact+normal for the linear cast (sweep) between two moving objects.
     *  Precondition is that objects should not penetration/overlap at the start from the interval. Overlap can be tested using GjkPairDetector.  */
    override fun calcTimeOfImpact(fromA: Transform, toA: Transform, fromB: Transform, toB: Transform, result: CastResult): Boolean {

        simplexSolver.reset()

        val linVelA = toA.origin - fromA.origin
        val linVelB = toB.origin - fromB.origin

        var lambda = 0f

        val interpolatedTransA = Transform(fromA)
        val interpolatedTransB = Transform(fromB)

        // take relative motion
        val r = linVelA - linVelB

        val supVertexA = fromA(convexA.localGetSupportingVertex(-r * fromA.basis))
        val supVertexB = fromB(convexB.localGetSupportingVertex(r * fromB.basis))
        val v = supVertexA - supVertexB
        var maxIter = MAX_ITERATIONS

        val n = Vec3()

        var dist2 = v.length2()
        val epsilon = 0.0001f
        val w = Vec3()
        val p = Vec3()
        var vDotR = 0f

        while (dist2 > epsilon && maxIter != 0) {
            maxIter--
            supVertexA put interpolatedTransA(convexA.localGetSupportingVertex(-v * interpolatedTransA.basis))
            supVertexB put interpolatedTransB(convexB.localGetSupportingVertex(v * interpolatedTransB.basis))
            w put supVertexA - supVertexB

            val vDotW = v dot w

            if (lambda > 1f) return false

            if (vDotW > 0f) {
                vDotR = v dot r
                if (vDotR >= -Float.EPSILON * Float.EPSILON) return false
                else {
                    lambda -= vDotW / vDotR
                    //interpolate to next lambda
                    //	x = s + lambda * r;
                    interpolatedTransA.origin.setInterpolate3(fromA.origin, toA.origin, lambda)
                    interpolatedTransB.origin.setInterpolate3(fromB.origin, toB.origin, lambda)
                    //m_simplexSolver->reset();
                    //check next line
                    w put supVertexA - supVertexB
                    n put v
                }
            }
            // Just like regular GJK only add the vertex if it isn't already (close) to current vertex, it would lead to divisions by zero and NaN etc.
            if (!simplexSolver.inSimplex(w))
                simplexSolver.addVertex(w, supVertexA, supVertexB)

            dist2 = if (simplexSolver.closest(v)) v.length2()
            //todo: check this normal for validity
            //n=v;
            //printf("V=%f , %f, %f\n",v[0],v[1],v[2]);
            //printf("DIST2=%f\n",dist2);
            //printf("numverts = %i\n",m_simplexSolver->numVertices());
            else 0f
        }
        //int numiter = MAX_ITERATIONS - maxIter;
//	printf("number of iterations: %d", numiter);
        // don't report a time of impact when moving 'away' from the hitnormal
        result.fraction = lambda
        if (n.length2() >= Float.EPSILON * Float.EPSILON)
            result.normal put n.normalized()
        else result.normal put 0f
        // don't report time of impact for motion away from the contact normal (or causes minor penetration)
        if (result.normal dot r >= -result.allowedPenetration) return false
        val hitA = Vec3()
        val hitB = Vec3()
        simplexSolver.computePoints(hitA, hitB)
        result.hitPoint put hitB
        return true
    }
}