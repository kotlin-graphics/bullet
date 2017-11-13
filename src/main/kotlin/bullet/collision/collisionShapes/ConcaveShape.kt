/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

package bullet.collision.collisionShapes

import bullet.linearMath.Vec3

/** PHY_ScalarType enumerates possible scalar types.
 *  See the StridingMeshInterface or btHeightfieldTerrainShape for its use  */
enum class PHY_ScalarType { NULL, FLOAT, DOUBLE, INTEGER, SHORT, FIXEDPOINT88, UCHAR;

    val i = ordinal - 1
}

/** The ConcaveShape class provides an interface for non-moving (static) concave shapes.
 *  It has been implemented by the StaticPlaneShape, BvhTriangleMeshShape and HeightfieldTerrainShape.  */
abstract class ConcaveShape : CollisionShape() {

    var collisionMargin = 0f

    abstract fun processAllTriangles(callback: TriangleCallback, aabbMin: Vec3, aabbMax: Vec3): Unit

    override var margin
        set(value) {
            collisionMargin = value
        }
        get() = collisionMargin
}