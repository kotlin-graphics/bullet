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

import bullet.has
import java.util.*

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
interface OverlappingPairCache : OverlappingPairCallback {

    val overlappingPairArray: Stack<BroadphasePair>

    fun cleanOverlappingPair(pair: BroadphasePair, dispatcher: Dispatcher)

    val numOverlappingPairs: Int

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

    val overlapFilterCallback: OverlapFilterCallback?

    fun processAllOverlappingPairs(callback: OverlapCallback, dispatcher: Dispatcher)

    fun findPair(proxy0: BroadphaseProxy, proxy1: BroadphaseProxy): BroadphasePair?

    val hasDeferredRemoval: Boolean

    val ghostPairCallback: OverlappingPairCallback?

    fun sortOverlappingPairs(dispatcher: Dispatcher)

    override fun removeOverlappingPairsContainingProxy(proxy0: BroadphaseProxy, dispatcher: Dispatcher) {
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

class HashedOverlappingPairCache : OverlappingPairCache {

    override var overlappingPairArray = Stack<BroadphasePair>()
    override var overlapFilterCallback: OverlapFilterCallback? = null

    var hashTable = intArrayOf()
    var next = intArrayOf()
    override var ghostPairCallback: OverlappingPairCallback? = null

    init {
//        int initialAllocatedSize= 2; TODO
//        m_overlappingPairArray.reserve(initialAllocatedSize);
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

        val hash = getHash(proxyId1, proxyId2) and (overlappingPairArray.size - 1)

        val pair = internalFindPair(_proxy0, _proxy1, hash) ?: return null

        cleanOverlappingPair(pair, dispatcher)

        val userData = pair.internalInfo1

        assert(pair.proxy0!!.uniqueId == proxyId1)
        assert(pair.proxy1!!.uniqueId == proxyId2)

        val pairIndex = overlappingPairArray.indexOf(pair)
        assert(pairIndex != -1)

        // Remove the pair from the hash table.
        var index = hashTable[hash]
        //assert(index != NULL_PAIR) means no getOrElse(_, NULL_PAIR)

        var previous = NULL_PAIR
        while (index != pairIndex) {
            previous = index
            index = next.getOrElse(index, { NULL_PAIR })
        }

        if (previous != NULL_PAIR) {
            assert(next[previous] == pairIndex)
            next[previous] = next[pairIndex]
        } else
            hashTable[hash] = next[pairIndex]

        /** We now move the last pair into spot of the pair being removed.
         *  We need to fix the hash table indices to support the move.  */
        val lastPairIndex = overlappingPairArray.lastIndex

        ghostPairCallback?.removeOverlappingPair(_proxy0, _proxy1, dispatcher)

        // If the removed pair is the last pair, we are done.
        if (lastPairIndex == pairIndex) {
            overlappingPairArray.pop()
            return userData
        }

        // Remove the last pair from the hash table.
        val last = overlappingPairArray[lastPairIndex]
        /* missing swap here too, Nat. */
        val lastHash = getHash(last.proxy0!!.uniqueId, last.proxy1!!.uniqueId) and overlappingPairArray.lastIndex   // TODO check (m_overlappingPairArray.capacity()-1)

        index = hashTable[lastHash]
        //assert(index != BT_NULL_PAIR) means no getOrElse

        previous = NULL_PAIR
        while (index != lastPairIndex) {
            previous = index
            index = next.getOrElse(index, { NULL_PAIR })
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
//        BT_PROFILE("btHashedOverlappingPairCache::processAllOverlappingPairs"); TODO
        var i = 0
//	printf("m_overlappingPairArray.size()=%d\n",m_overlappingPairArray.size());
        while (i < overlappingPairArray.size) {
            val pair = overlappingPairArray[i]
            if (callback.processOverlap(pair)) {
                removeOverlappingPair(pair.proxy0!!, pair.proxy1!!, dispatcher)
                overlappingPairs--
            } else i++
        }
    }

    override fun cleanOverlappingPair(pair: BroadphasePair, dispatcher: Dispatcher) {
        pair.algorithm?.let {
            dispatcher.freeCollisionAlgorithm(pair.algorithm)
            pair.algorithm = null
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
        val proxyId1 = proxy0.uniqueId
        val proxyId2 = proxy1.uniqueId

        val hash = getHash(proxyId1, proxyId2) and overlappingPairArray.lastIndex

        if (hash >= hashTable.size) return null

        var index = hashTable.getOrElse(hash, { NULL_PAIR })
        while (index != NULL_PAIR && !equalsPair(overlappingPairArray[index], proxyId1, proxyId2)) {
            index = next.getOrElse(index, { NULL_PAIR })
        }

        if (index == NULL_PAIR) return null

        return overlappingPairArray[index]
    }

    val count get() = overlappingPairArray.size
    override val numOverlappingPairs get() = overlappingPairArray.size

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

        val hash = getHash(proxyId1, proxyId2) and overlappingPairArray.lastIndex    // New hash value with new mask

        val pair = internalFindPair(proxy0, proxy1, hash)
        if (pair != null)
            return pair

        TODO()
//        val count = overlappingPairArray.size
//        val oldCapacity = m_overlappingPairArray . capacity ()
//        void * mem = & m_overlappingPairArray . expandNonInitializing ()
//
//        //this is where we add an actual pair, so also call the 'ghost'
//        if (m_ghostPairCallback)
//            m_ghostPairCallback->addOverlappingPair(proxy0, proxy1)
//
//        int newCapacity = m_overlappingPairArray . capacity ()
//
//        if (oldCapacity < newCapacity) {
//            growTables()
//            //hash with new capacity
//            hash = static_cast<int>(getHash(static_cast < unsigned int >(proxyId1), static_cast < unsigned int >(proxyId2)) &(m_overlappingPairArray.capacity() - 1))
//        }
//
//        pair = new(mem) btBroadphasePair ( * proxy0, *proxy1)
////	pair->m_pProxy0 = proxy0;
////	pair->m_pProxy1 = proxy1;
//        pair->m_algorithm = 0
//        pair->m_internalTmpValue = 0
//
//
//        m_next[count] = m_hashTable[hash]
//        m_hashTable[hash] = count
//
//        return pair
    }

    void    growTables()

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

        while (index != NULL_PAIR && !equalsPair(overlappingPairArray[index], proxyId1, proxyId2))
            index = next.getOrElse(index, { NULL_PAIR })

        if (index == NULL_PAIR) return null

        return overlappingPairArray[index]
    }

    virtual bool    hasDeferredRemoval()
    {
        return false
    }

    virtual    void    setInternalGhostPairCallback(btOverlappingPairCallback* ghostPairCallback)
    {
        m_ghostPairCallback = ghostPairCallback
    }

    virtual void    sortOverlappingPairs(btDispatcher* dispatcher)


}