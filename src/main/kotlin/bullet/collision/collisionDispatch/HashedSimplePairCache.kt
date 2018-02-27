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

import bullet.linearMath.AlignedSimplePairArray

//import bullet.L
//
//class HashedSimplePairCache : HashMap<Long, Any?>() {
//
//    val Int.uL get() = L and 0xffffffffL
//
//    operator fun get(indexA: Int, indexB: Int): Any? {
//        val a = indexA.uL shl 32
//        val b = indexB.uL
//        return get(a and b)
//    }
//
//    operator fun set(indexA: Int, indexB: Int, value: Any?) {
//        val a = indexA.uL shl 32
//        val b = indexB.uL
//        return set(a and b, value)
//    }
//}

var gOverlappingSimplePairs = 0
var gRemoveSimplePairs = 0
var gAddedSimplePairs = 0
var gFindSimplePairs = 0

val NULL_PAIR = -1

class SimplePair(val indexA: Int, val indexB: Int) {
    var userPointer: Any? = null
    var userValue
        get() = userPointer as? Int
        set(value) {
            userPointer = value
        }
}

class HashedSimplePairCache {

    val initialAllocatedSize = 2
    val overlappingPairArray = AlignedSimplePairArray().apply { reserve(initialAllocatedSize) }

    init {
        growTables()
    }

    var hashTable = intArrayOf()
    var next = intArrayOf()

    fun removeAllPairs() {
        overlappingPairArray.clear()
        hashTable = intArrayOf()
        next = intArrayOf()

        overlappingPairArray reserve initialAllocatedSize
        growTables()
    }

    fun removeOverlappingPair(indexA: Int, indexB: Int): Any? {

        gRemoveSimplePairs++

        // if (indexA > indexB) btSwap(indexA, indexB)

        val hash = getHash(indexA, indexB) and (overlappingPairArray.capacity - 1)

        val pair = internalFindPair(indexA, indexB, hash) ?: return null

        val userData = pair.userPointer

        val pairIndex = overlappingPairArray indexOf pair
        assert(pairIndex < overlappingPairArray.size)

        // Remove the pair from the hash table.
        var index = hashTable[hash]

        var previous = NULL_PAIR
        while (index != pairIndex) {
            previous = index
            index = next[index]
        }

        if (previous != NULL_PAIR) {
            assert(next[previous] == pairIndex)
            next[previous] = next[pairIndex]
        } else
            hashTable[hash] = next[pairIndex]

        // We now move the last pair into spot of the pair being removed. We need to fix the hash table indices to support the move.
        val lastPairIndex = overlappingPairArray.lastIndex

        // If the removed pair is the last pair, we are done.
        if (lastPairIndex == pairIndex) {
            overlappingPairArray.pop()
            return userData
        }

        // Remove the last pair from the hash table.
        val last = overlappingPairArray[lastPairIndex]!!
        // missing swap here too, Nat.
        val lastHash = getHash(last.indexA, last.indexB) and (overlappingPairArray.capacity-1)

        index = hashTable[lastHash]

        previous = NULL_PAIR
        while (index != lastPairIndex) {
            previous = index
            index = next[index]
        }

        if (previous != NULL_PAIR) {
            assert(next[previous] == lastPairIndex)
            next[previous] = next[lastPairIndex]
        } else
            hashTable[lastHash] = next[lastPairIndex]

        // Copy the last pair into the remove pair's spot.
        overlappingPairArray[pairIndex] = overlappingPairArray[lastPairIndex]

        // Insert the last pair into the hash table
        next[pairIndex] = hashTable[lastHash]
        hashTable[lastHash] = pairIndex

        overlappingPairArray.pop()

        return userData
    }

    /** Add a pair and return the new pair. If the pair already exists, no new pair is created and the old one is returned. */
    fun addOverlappingPair(indexA: Int, indexB: Int): SimplePair? {
        gAddedSimplePairs++
        return internalAddPair(indexA, indexB)
    }

    fun findPair(indexA: Int, indexB: Int): SimplePair? {

        gFindSimplePairs++

        //if (indexA > indexB) btSwap(indexA, indexB)

        val hash = getHash(indexA, indexB) and (overlappingPairArray.capacity - 1)

        if (hash >= hashTable.size) return null

        var index = hashTable.getOrElse(hash) { NULL_PAIR }
        while (index != NULL_PAIR && !equalsPair(overlappingPairArray[index]!!, indexA, indexB))
            index = next[index]

        if (index == NULL_PAIR) return null

        assert(index < overlappingPairArray.size)

        return overlappingPairArray[index]
    }

    val count get() = overlappingPairArray.size

    val numOverlappingPairs get() = overlappingPairArray.size

    fun internalAddPair(indexA: Int, indexB: Int): SimplePair? {

        var hash = getHash(indexA, indexB) and (overlappingPairArray.capacity - 1)    // New hash value with new mask

        var pair = internalFindPair(indexA, indexB, hash)
        if (pair != null) return pair

        val count = overlappingPairArray.size
        val oldCapacity = overlappingPairArray.capacity
        pair = SimplePair(indexA, indexB)
        overlappingPairArray += pair

        val newCapacity = overlappingPairArray.capacity

        if (oldCapacity < newCapacity) {
            growTables()

            hash = getHash(indexA, indexB) and (overlappingPairArray.capacity - 1) // hash with new capacity
        }

        next[count] = hashTable[hash]
        hashTable[hash] = count

        return pair
    }


    fun growTables() {

        val newCapacity = overlappingPairArray.capacity

        if (hashTable.size < newCapacity) {

            //grow hashtable and next table
            val curHashtableSize = hashTable.size

            hashTable = IntArray(newCapacity) { NULL_PAIR }
            next = IntArray(newCapacity) { NULL_PAIR }

            for (i in 0 until curHashtableSize) {

                val pair = overlappingPairArray[i]!!
                val indexA = pair.indexA
                val indexB = pair.indexB

                // New hash value with new mask
                val hashValue = getHash(indexA, indexB) and (overlappingPairArray.capacity - 1)
                next[i] = hashTable[hashValue]
                hashTable[hashValue] = i
            }
        }
    }

    fun equalsPair(pair: SimplePair, indexA: Int, indexB: Int) = pair.indexA == indexA && pair.indexB == indexB

    fun getHash(indexA: Int, indexB: Int): Int {

        var key = indexA or (indexB shl 16)
        // Thomas Wang's hash

        key += (key shl 15).inv()
        key = key xor (key ushr 10)
        key += key shl 3
        key = key xor (key ushr 6)
        key += (key shl 11).inv()
        key = key xor (key ushr 16)
        return key
    }

    fun internalFindPair(proxyIdA: Int, proxyIdB: Int, hash: Int): SimplePair? {

        var index = hashTable.getOrElse(hash) { NULL_PAIR }

        while (index != NULL_PAIR && !equalsPair(overlappingPairArray[index]!!, proxyIdA, proxyIdB))
            index = next[index]

        if (index == NULL_PAIR) return null

        assert(index < overlappingPairArray.size)

        return overlappingPairArray[index]
    }
}
