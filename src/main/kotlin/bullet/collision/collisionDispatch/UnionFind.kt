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

import bullet.USE_PATH_COMPRESSION
import bullet.pop

/** see for discussion of static island optimizations by Vroonsh here: http://code.google.com/p/bullet/issues/detail?id=406 */
val STATIC_SIMULATION_ISLAND_OPTIMIZATION = true

class Element {
    var id = 0
    var sz = 0
}

/** UnionFind calculates connected subsets
 *  Implements weighted Quick Union with path compression
 *  optimization: could use short ints instead of ints (halving memory, would limit the number of rigid bodies to 64k,
 *  sounds reasonable) */
class UnionFind {

    val elements = ArrayList<Element>()

    /** this is a special operation, destroying the content of UnionFind.
     *  it sorts the elements, based on island id, in order to make it easy to iterate over islands */
    fun sortIslands() = elements.sortBy { it.id }

    fun reset(n: Int) {
        allocate(n)
        for (i in 0 until n) {
            elements[i].id = i
            elements[i].sz = 1
        }
    }

    val numElements get() = elements.size
    fun isRoot(x: Int) = x == elements[x].id

    operator fun get(index: Int) = elements[index]

    fun allocate(n: Int) {
        if (elements.size < n)
            for (i in elements.size until n)
                elements.add(Element())
        else if (elements.size > n)
            for (i in n until elements.size)
                elements.pop()
    }

    fun free() = elements.clear()

    fun find(p: Int, q: Int) = find(p) == find(q)

    fun unite(p: Int, q: Int) {
        val i = find(p)
        val j = find(q)
        if (i == j) return

        if (!USE_PATH_COMPRESSION)
        // weighted quick union, this keeps the 'trees' balanced, and keeps performance of unite O( log(n) )
            if (elements[i].sz < elements[j].sz) {
                elements[i].id = j
                elements[j].sz += elements[i].sz
            } else {
                elements[j].id = i
                elements[i].sz += elements[j].sz
            }
        else {
            elements[i].id = j
            elements[j].sz += elements[i].sz
        }
    }

    fun find(x: Int): Int {
        //btAssert(x < m_N);
        //btAssert(x >= 0);
        var x = x
        while (x != elements[x].id) {
            //not really a reason not to use path compression, and it flattens the trees/improves find performance dramatically
            if (USE_PATH_COMPRESSION) {
                val elementPtr = elements[elements[x].id]
                elements[x].id = elementPtr.id
                x = elementPtr.id
            } else
                x = elements[x].id
            //btAssert(x < m_N);
            //btAssert(x >= 0);
        }
        return x
    }
}