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

import bullet.collision.broadphaseCollision.BroadphasePair


class AlignedBroadphasePairArray() {

    /** return the number of elements in the array  */
    var size = 0
    /** return the pre-allocated (reserved) elements, this is at least as large as the total number of elements,
     *  see ::size and reserve()    */
    var capacity = 0
    var data = Array<BroadphasePair?>(0, { null })

    //PCK: added this line
    var ownsMemory = true

    constructor(other: AlignedBroadphasePairArray) : this() {
        put(other)
    }

    infix fun put(other: AlignedBroadphasePairArray) {
        val otherSize = other.size
        resize(otherSize)
        other.copy(0, otherSize, data)
    }

    fun allocSize(size: Int) = if (size != 0) size * 2 else 1

    fun copy(start: Int, end: Int, dest: Array<BroadphasePair?>, ptr: Int = 0) {
        for (i in start until end)
            dest[ptr + i] = data[i]
    }

    fun allocate(size: Int) {
        if (size != 0) data = Array(size, { null })
    }

//    fun deallocate() {
//        if (data.isNotEmpty())
//        //PCK: enclosed the deallocation in this block
//            if (ownsMemory)
//                data = intArrayOf()
//    }

    fun at(n: Int) = data[n]

    operator fun get(n: Int) = data.getOrNull(n)
    operator fun set(n: Int, pair: BroadphasePair?) = data.set(n, pair)
    operator fun plusAssign(pair: BroadphasePair) = push(pair)

    fun init() {
        size = 0
        capacity = 0
        data = arrayOf()
        ownsMemory = true
    }

    fun pop() {
        assert(size > 0)
        size--
    }

    /** resize changes the number of elements in the array. If the new size is larger, the new elements will be
     *  constructed using the optional second argument.
     *  when the new number of elements is smaller, the destructor will be called, but memory will not be freed,
     *  to reduce performance overhead of run-time memory (de)allocations.  */
    fun resizeNoInitialize(newSize: Int) {
        if (newSize > size)
            reserve(newSize)
        size = newSize
    }

    fun resize(newSize: Int, fillData: BroadphasePair? = null) {

        val curSize = size

        if (newSize > curSize) {

            reserve(newSize)

            for (i in curSize until newSize)
                data[i] = fillData
        }
        size = newSize
    }

//    fun expandNonInitializing(): SimplePair {
//        val sz = size
//        if (sz == capacity)
//            reserve(allocSize(size))
//        size++
//
//        return data[sz]
//    }

    fun expand(fillValue: BroadphasePair = BroadphasePair()): BroadphasePair {

        val sz = size
        if (sz == capacity)
            reserve(allocSize(size))

        size++
        data[sz] = fillValue //use the in-place new (not really allocating heap memory)

        return fillValue
    }

    fun push(value: BroadphasePair) {

        val sz = size
        if (sz == capacity)
            reserve(allocSize(size))

        data[size] = value

        size++
    }


    fun reserve(count: Int) {
        // determine new minimum length of allocated storage
        if (capacity < count) {    // not enough room, reallocate
            val s = Array<BroadphasePair?>(count, { null })

            copy(0, size, s)

            //PCK: added this line
            ownsMemory = true
            data = s
            capacity = count
        }
    }

    fun quickSortInternal(compareFunc: (BroadphasePair, BroadphasePair)-> Boolean, lo: Int, hi: Int) {
        //  lo is the lower index, hi is the upper index of the region of array a that is to be sorted
        var i = lo
        var j = hi
        val x = data [(lo + hi) / 2]!!

        //  partition
        do {
            while (compareFunc(data[i]!!, x))
                i++
            while (compareFunc(x, data[j]!!))
                j--
            if (i <= j) {
                val t = i
                i = j
                j = t
                i++
                j--
            }
        } while (i <= j)

        //  recursion
        if (lo < j)
            quickSortInternal(compareFunc, lo, j)
        if (i < hi)
            quickSortInternal(compareFunc, i, hi)
    }


    infix fun quickSort(compareFunc: (BroadphasePair, BroadphasePair) -> Boolean) {
        //don't sort 0 or 1 elements
        if (size > 1)
            quickSortInternal(compareFunc, 0, size - 1)
    }

    fun indexOf(pair: BroadphasePair) = data.indexOf(pair)
}