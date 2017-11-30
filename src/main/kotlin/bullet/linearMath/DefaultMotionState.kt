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

package bullet.linearMath

/** The DefaultMotionState provides a common implementation to synchronize world transforms with offsets. */
class DefaultMotionState(val startWorldTrans: Transform = Transform.identity,
                         val centerOfMassOffset: Transform = Transform.identity) : MotionState {

    val graphicsWorldTrans = Transform(startWorldTrans)
    var userPointer: Any? = null

    /** synchronizes world transform from user to physics */
    override fun getWorldTransform(worldTransform: Transform ) = worldTransform put graphicsWorldTrans * centerOfMassOffset.inverse()

    /** synchronizes world transform from physics to user
     *  Bullet only calls the update of worldtransform for active objects */
    override fun setWorldTransform(worldTransform: Transform) = graphicsWorldTrans put worldTransform * centerOfMassOffset // TODO check
}