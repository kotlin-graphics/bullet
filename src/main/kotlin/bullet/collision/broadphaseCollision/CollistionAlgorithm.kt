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

package bullet.collision.broadphaseCollision

import bullet.collision.collisionDispatch.CollisionObject
import bullet.collision.collisionDispatch.CollisionObjectWrapper
import bullet.collision.collisionDispatch.ManifoldResult
import bullet.collision.narrowPhaseCollision.PersistentManifold

class CollisionAlgorithmConstructionInfo(var dispatcher: Dispatcher? = null, var manifold: PersistentManifold? = null)

/** CollisionAlgorithm is an collision interface that is compatible with the Broadphase and Dispatcher.
 *  It is persistent over frames    */
abstract class CollisionAlgorithm {

    var dispatcher: Dispatcher? = null

    constructor()
    constructor(ci: CollisionAlgorithmConstructionInfo) {
        dispatcher = ci.dispatcher
    }

    open fun processCollision(body0Wrap: CollisionObjectWrapper, body1Wrap: CollisionObjectWrapper, dispatchInfo: DispatcherInfo,
                                  resultOut: ManifoldResult) = Unit

    open fun calculateTimeOfImpact(body0: CollisionObject, body1: CollisionObject, dispatchInfo: DispatcherInfo,
                                       resultOut: ManifoldResult) = 0f

    open fun getAllContactManifolds(manifoldArray: ArrayList<PersistentManifold>) = Unit
}