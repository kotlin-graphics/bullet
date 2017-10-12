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

/** The OverlappingPairCallback interface is an additional optional broadphase user callback for adding/removing
 *  overlapping pairs, similar interface to OverlappingPairCache. */
interface OverlappingPairCallback{

    fun addOverlappingPair(proxy0: BroadphaseProxy, proxy1:BroadphaseProxy): BroadphasePair?

    fun removeOverlappingPair(proxy0:BroadphaseProxy,proxy1:BroadphaseProxy,dispatcher:Dispatcher): Any?

    fun removeOverlappingPairsContainingProxy(proxy0:BroadphaseProxy,dispatcher:Dispatcher)
}