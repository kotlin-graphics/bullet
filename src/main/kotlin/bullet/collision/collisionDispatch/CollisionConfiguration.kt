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

package bullet.collision.collisionDispatch

import bullet.collision.broadphaseCollision.BroadphaseNativeTypes

/** CollisionConfiguration allows to configure Bullet collision detection
 *  stack allocator size, default collision algorithms and persistent manifold pool size
 *  todo: describe the meaning  */
abstract class CollisionConfiguration {

    ///memory pools TODO
//    virtual btPoolAllocator* getPersistentManifoldPool() = 0;
//
//    virtual btPoolAllocator* getCollisionAlgorithmPool() = 0;

    abstract fun getCollisionAlgorithmCreateFunc(proxyType0: BroadphaseNativeTypes, proxyType1: BroadphaseNativeTypes): CollisionAlgorithmCreateFunc
    fun getCollisionAlgorithmCreateFunc(proxyType0: Int, proxyType1: Int) =
            getCollisionAlgorithmCreateFunc(BroadphaseNativeTypes.of(proxyType0), BroadphaseNativeTypes.of(proxyType1))

    abstract fun getClosestPointsAlgorithmCreateFunc(proxyType0: BroadphaseNativeTypes, proxyType1: BroadphaseNativeTypes): CollisionAlgorithmCreateFunc
    fun getClosestPointsAlgorithmCreateFunc(proxyType0: Int, proxyType1: Int) =
            getClosestPointsAlgorithmCreateFunc(BroadphaseNativeTypes.of(proxyType0), BroadphaseNativeTypes.of(proxyType1))
}