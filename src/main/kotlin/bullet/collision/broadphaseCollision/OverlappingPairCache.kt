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

import bullet.BT_PROFILE
import bullet.has
import bullet.linearMath.AlignedBroadphasePairArray
import bullet.push

var overlappingPairs = 0

var removePairs = 0
var addedPairs = 0
var findPairs = 0

val NULL_PAIR = -1

interface OverlapCallback {
    /** return true for deletion of the pair    */
    fun processOverlap(pair: BroadphasePair): Boolean
}

interface OverlapFilterCallback {
    /** return true when pairs need collision   */
    fun needBroadphaseCollision(proxy0: BroadphaseProxy, proxy1: BroadphaseProxy): Boolean
}

/** The OverlappingPairCache provides an interface for overlapping pair management (add, remove, storage),
 *  used by the BroadphaseInterface broadphases.
 *  The HashedOverlappingPairCache and SortedOverlappingPairCache classes are two implementations.  */
abstract class OverlappingPairCache : OverlappingPairCallback {

    abstract val overlappingPairArray: AlignedBroadphasePairArray

    abstract fun cleanOverlappingPair(pair: BroadphasePair, dispatcher: Dispatcher?)

    val numOverlappingPairs get() = overlappingPairArray.size

    fun cleanProxyFromPairs(proxy: BroadphaseProxy, dispatcher: Dispatcher) {
        class CleanPairCallback(val cleanProxy: BroadphaseProxy, val pairCache: OverlappingPairCache, val dispatcher: Dispatcher)
            : OverlapCallback {
            override fun processOverlap(pair: BroadphasePair): Boolean {
                if (pair.proxy0 === cleanProxy || pair.proxy1 === cleanProxy)
                    pairCache.cleanOverlappingPair(pair, dispatcher)
                return false
            }
        }

        val cleanPairs = CleanPairCallback(proxy, this, dispatcher)
        processAllOverlappingPairs(cleanPairs, dispatcher)
    }

    abstract val overlapFilterCallback: OverlapFilterCallback?

    abstract fun processAllOverlappingPairs(callback: OverlapCallback, dispatcher: Dispatcher)

    abstract fun findPair(proxy0: BroadphaseProxy, proxy1: BroadphaseProxy): BroadphasePair?

    abstract val hasDeferredRemoval: Boolean

    abstract val ghostPairCallback: OverlappingPairCallback?

    abstract fun sortOverlappingPairs(dispatcher: Dispatcher)

    override final fun removeOverlappingPairsContainingProxy(proxy0: BroadphaseProxy, dispatcher: Dispatcher) {
        class RemovePairCallback(val obsoleteProxy: BroadphaseProxy) : OverlapCallback {
            override fun processOverlap(pair: BroadphasePair) = pair.proxy0 === obsoleteProxy || pair.proxy1 === obsoleteProxy
        }

        val removeCallback = RemovePairCallback(proxy0)
        processAllOverlappingPairs(removeCallback, dispatcher)
    }

    fun needsBroadphaseCollision(proxy0: BroadphaseProxy, proxy1: BroadphaseProxy): Boolean {
        overlapFilterCallback?.let { return it.needBroadphaseCollision(proxy0, proxy1) }
        // collides?
        return proxy0.collisionFilterGroup has proxy1.collisionFilterMask && proxy1.collisionFilterGroup has proxy0.collisionFilterMask
    }
}

class HashedOverlappingPairCache : OverlappingPairCache() {

    override val overlappingPairArray = AlignedBroadphasePairArray().apply { reserve(2) }
    override var overlapFilterCallback: OverlapFilterCallback? = null

    var hashTable = intArrayOf()
    var next = intArrayOf()
    override var ghostPairCallback: OverlappingPairCallback? = null

    init {
        growTables()
    }

    override fun removeOverlappingPair(proxy0: BroadphaseProxy, proxy1: BroadphaseProxy, dispatcher: Dispatcher): Any? {

        removePairs++
        val _proxy0: BroadphaseProxy
        val _proxy1: BroadphaseProxy
        if (proxy0.uniqueId > proxy1.uniqueId) {
            _proxy0 = proxy1
            _proxy1 = proxy0
        } else {
            _proxy0 = proxy0
            _proxy1 = proxy1
        }
        val proxyId1 = _proxy0.uniqueId
        val proxyId2 = _proxy1.uniqueId

        val hash = getHash(proxyId1, proxyId2) and (overlappingPairArray.capacity - 1)

        val pair = internalFindPair(_proxy0, _proxy1, hash) ?: return null

        cleanOverlappingPair(pair, dispatcher)

        val userData = pair.internalInfo1

        assert(pair.proxy0!!.uniqueId == proxyId1)
        assert(pair.proxy1!!.uniqueId == proxyId2)

        val pairIndex = overlappingPairArray.indexOf(pair)
        assert(pairIndex != -1)

        // Remove the pair from the hash table.
        var index = hashTable[hash] // assert implicit

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

        /** We now move the last pair into spot of the pair being removed.
         *  We need to fix the hash table indices to support the move.  */
        val lastPairIndex = overlappingPairArray.size - 1

        ghostPairCallback?.removeOverlappingPair(_proxy0, _proxy1, dispatcher)

        // If the removed pair is the last pair, we are done.
        if (lastPairIndex == pairIndex) {
            overlappingPairArray.pop()
            return userData
        }

        // Remove the last pair from the hash table.
        val last = overlappingPairArray[lastPairIndex]!!
        /* missing swap here too, Nat. */
        val lastHash = getHash(last.proxy0!!.uniqueId, last.proxy1!!.uniqueId) and (overlappingPairArray.capacity - 1)

        index = hashTable[lastHash] // assert implicit

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
    override fun addOverlappingPair(proxy0: BroadphaseProxy, proxy1: BroadphaseProxy): BroadphasePair? {
        addedPairs++
        return if (!needsBroadphaseCollision(proxy0, proxy1)) null else internalAddPair(proxy0, proxy1)
    }

    override fun processAllOverlappingPairs(callback: OverlapCallback, dispatcher: Dispatcher) {
        BT_PROFILE("HashedOverlappingPairCache::processAllOverlappingPairs")
        var i = 0
//	printf("m_overlappingPairArray.size()=%d\n",m_overlappingPairArray.size());
        while (i < overlappingPairArray.size) {
            val pair = overlappingPairArray[i]!!
            if (callback.processOverlap(pair)) {
                removeOverlappingPair(pair.proxy0!!, pair.proxy1!!, dispatcher)
                overlappingPairs--
            } else i++
        }
    }

    override fun cleanOverlappingPair(pair: BroadphasePair, dispatcher: Dispatcher?) {
        pair.algorithm?.let {
            if (dispatcher != null) {
                dispatcher.freeCollisionAlgorithm(it)
                pair.algorithm = null
            }
        }
    }

    override fun findPair(proxy0: BroadphaseProxy, proxy1: BroadphaseProxy): BroadphasePair? {
        findPairs++
        val _proxy0: BroadphaseProxy
        val _proxy1: BroadphaseProxy
        if (proxy0.uniqueId > proxy1.uniqueId) {
            _proxy0 = proxy1
            _proxy1 = proxy0
        } else {
            _proxy0 = proxy0
            _proxy1 = proxy1
        }
        val proxyId1 = _proxy0.uniqueId
        val proxyId2 = _proxy1.uniqueId

        val hash = getHash(proxyId1, proxyId2) and (overlappingPairArray.capacity - 1)

        if (hash >= hashTable.size) return null

        var index = hashTable.getOrElse(hash, { NULL_PAIR })
        while (index != NULL_PAIR && !equalsPair(overlappingPairArray[index]!!, proxyId1, proxyId2)) {
            index = next[index]
        }

        return overlappingPairArray[index].takeIf { index != NULL_PAIR }
    }

    val count get() = overlappingPairArray.size

    fun internalAddPair(proxy0: BroadphaseProxy, proxy1: BroadphaseProxy): BroadphasePair {
        val _proxy0: BroadphaseProxy
        val _proxy1: BroadphaseProxy
        if (proxy0.uniqueId > proxy1.uniqueId) {
            _proxy0 = proxy1
            _proxy1 = proxy0
        } else {
            _proxy0 = proxy0
            _proxy1 = proxy1
        }
        val proxyId1 = _proxy0.uniqueId
        val proxyId2 = _proxy1.uniqueId

        var hash = getHash(proxyId1, proxyId2) and (overlappingPairArray.capacity - 1)    // New hash value with new mask

        var pair = internalFindPair(_proxy0, _proxy1, hash)
        if (pair != null)
            return pair

        val count = overlappingPairArray.size
        val oldCapacity = overlappingPairArray.capacity
        pair = BroadphasePair(_proxy0, _proxy1)
        overlappingPairArray += pair

        //this is where we add an actual pair, so also call the 'ghost'
        ghostPairCallback?.addOverlappingPair(_proxy0, _proxy1)

        val newCapacity = overlappingPairArray.capacity

        if (oldCapacity < newCapacity) {
            growTables()
            // hash with new capacity
            hash = getHash(proxyId1, proxyId2) and (overlappingPairArray.capacity - 1)
        }

//	pair->m_pProxy0 = proxy0;
//	pair->m_pProxy1 = proxy1;

        next[count] = hashTable[hash]
        hashTable[hash] = count

        return pair
    }

    fun growTables() {

        val newCapacity = overlappingPairArray.capacity

        if (hashTable.size < newCapacity) {
            //grow hashtable and next table
            val curHashtableSize = hashTable.size

            hashTable = IntArray(newCapacity, { NULL_PAIR })
            next = IntArray(newCapacity, { NULL_PAIR })

            for (i in 0 until curHashtableSize) {
                val pair = overlappingPairArray[i]!!
                val proxyId1 = pair.proxy0!!.uniqueId
                val proxyId2 = pair.proxy1!!.uniqueId
                val hashValue = getHash(proxyId1, proxyId2) and (overlappingPairArray.capacity - 1)    // New hash value with new mask
                next[i] = hashTable[hashValue]
                hashTable[hashValue] = i
            }
        }
    }

    fun equalsPair(pair: BroadphasePair, proxyId1: Int, proxyId2: Int) = pair.proxy0!!.uniqueId == proxyId1 && pair.proxy1!!.uniqueId == proxyId2

    /*
    // Thomas Wang's hash, see: http://www.concentric.net/~Ttwang/tech/inthash.htm
    // This assumes proxyId1 and proxyId2 are 16-bit.
    SIMD_FORCE_INLINE int getHash(int proxyId1, int proxyId2)
    {
        int key = (proxyId2 << 16) | proxyId1;
        key = ~key + (key << 15);
        key = key ^ (key >> 12);
        key = key + (key << 2);
        key = key ^ (key >> 4);
        key = key * 2057;
        key = key ^ (key >> 16);
        return key;
    }
    */

    fun getHash(proxyId1: Int, proxyId2: Int): Int {
        var key = proxyId1 or (proxyId2 shl 16)
        // Thomas Wang's hash
        key += (key shl 15).inv()
        key = key xor (key ushr 10)
        key += key shl 3
        key = key xor (key ushr 6)
        key += (key shl 11).inv()
        key = key xor (key ushr 16)
        return key
    }

    fun internalFindPair(proxy0: BroadphaseProxy, proxy1: BroadphaseProxy, hash: Int): BroadphasePair? {
        val proxyId1 = proxy0.uniqueId
        val proxyId2 = proxy1.uniqueId

        var index = hashTable.getOrElse(hash, { NULL_PAIR })

        while (index != NULL_PAIR && !equalsPair(overlappingPairArray[index]!!, proxyId1, proxyId2))
            index = next[index]

        return overlappingPairArray[index].takeIf { index != NULL_PAIR }
    }

    override val hasDeferredRemoval get() = false

    override fun sortOverlappingPairs(dispatcher: Dispatcher) {

        // need to keep hashmap in sync with pair address, so rebuild all
        val tmpPairs = AlignedBroadphasePairArray()
        var i = 0
        while (i < overlappingPairArray.size)
            tmpPairs += overlappingPairArray[i]!!
        i = 0
        while (i < tmpPairs.size)
            removeOverlappingPair(tmpPairs[i]!!.proxy0!!, tmpPairs[i]!!.proxy1!!, dispatcher)
        i = 0
        while (i < next.size) next[i] = NULL_PAIR

        tmpPairs quickSort ::BroadphasePairSortPredicate
        i = 0
        while (i < tmpPairs.size)
            addOverlappingPair(tmpPairs[i]!!.proxy0!!, tmpPairs[i]!!.proxy1!!)
    }
}


/** SortedOverlappingPairCache maintains the objects with overlapping AABB
 *  Typically managed by the Broadphase, Axis3Sweep or SimpleBroadphase   */
//class SortedOverlappingPairCache : OverlappingPairCache() {
//
//    /** avoid brute-force finding all the time  */
//    override var overlappingPairArray = arrayListOf(BroadphasePair(), BroadphasePair()) // initialAllocatedSize = 2
//
//    /** during the dispatch, check that user doesn't destroy/create proxy   */
//    var blockedForChanges = false
//
//    /** by default, do the removal during the pair traversal    */
//    override var hasDeferredRemoval = true
//
//    /** if set, use the callback instead of the built in filter in needBroadphaseCollision  */
//    override var overlapFilterCallback: OverlapFilterCallback? = null
//
//    override var ghostPairCallback: OverlappingPairCallback? = null
//
//    override fun processAllOverlappingPairs(callback: OverlapCallback, dispatcher: Dispatcher) {
//        var i = 0
//        while (i < overlappingPairArray.size) {
//            val pair = overlappingPairArray[i]
//            if (callback.processOverlap(pair)) {
//                cleanOverlappingPair(pair, dispatcher)
//                pair.proxy0 = null
//                pair.proxy1 = null
//                overlappingPairArray.swapLastAt(i)
//                overlappingPairArray.pop()
//                overlappingPairs--
//            } else i++
//        }
//    }
//
//    override fun removeOverlappingPair(proxy0: BroadphaseProxy, proxy1: BroadphaseProxy, dispatcher: Dispatcher): Any? {
//        if (!hasDeferredRemoval) {
//            val findPair = BroadphasePair(proxy0, proxy1)
//
//            val findIndex = overlappingPairArray.indexOf(findPair)
//            if (findIndex < overlappingPairArray.size) {
//                overlappingPairs--
//                val pair = overlappingPairArray[findIndex]
//                val userData = pair.internalInfo1
//                cleanOverlappingPair(pair, dispatcher)
//                ghostPairCallback?.removeOverlappingPair(proxy0, proxy1, dispatcher)
//
//                overlappingPairArray.swap(findIndex, overlappingPairArray.lastIndex)
//                overlappingPairArray.pop()
//                return userData
//            }
//        }
//        return null
//    }
//
//    override fun cleanOverlappingPair(pair: BroadphasePair, dispatcher: Dispatcher) {
//        pair.algorithm?.let {
//            dispatcher.freeCollisionAlgorithm(pair.algorithm)
//            pair.algorithm = null
//            removePairs--
//        }
//    }
//
//    override fun addOverlappingPair(proxy0: BroadphaseProxy, proxy1: BroadphaseProxy): BroadphasePair? {
//        //don't add overlap with own
//        assert(proxy0 !== proxy1)
//
//        if (!needsBroadphaseCollision(proxy0, proxy1)) return null
//
//        val pair = BroadphasePair(proxy0, proxy1)
//        overlappingPairArray.add(pair)
//
//        overlappingPairs++
//        addedPairs++
//
//        ghostPairCallback?.addOverlappingPair(proxy0, proxy1)
//        return pair
//    }
//
//    /** This findPair becomes really slow. Either sort the list to speedup the query, or use a different solution.
//     *  It is mainly used for Removing overlapping pairs. Removal could be delayed.
//     *  We could keep a linked list in each proxy, and store pair in one of the proxies (with lowest memory address)
//     *  Also we can use a 2D bitmap, which can be useful for a future GPU implementation    */
//    override fun findPair(proxy0: BroadphaseProxy, proxy1: BroadphaseProxy): BroadphasePair? {
//        if (!needsBroadphaseCollision(proxy0, proxy1)) return null
//
//        val tmpPair = BroadphasePair(proxy0, proxy1)
//        val findIndex = overlappingPairArray.indexOf(tmpPair)
//
//        return overlappingPairArray.getOrNull(findIndex)
//    }
//
//    override fun sortOverlappingPairs(dispatcher: Dispatcher) = Unit  //should already be sorted
//}