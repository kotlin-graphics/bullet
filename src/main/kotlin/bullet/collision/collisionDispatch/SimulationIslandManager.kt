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

import bullet.BT_PROFILE
import bullet.collision.broadphaseCollision.Dispatcher
import bullet.collision.narrowPhaseCollision.PersistentManifold

/** SimulationIslandManager creates and handles simulation islands, using UnionFind */
class SimulationIslandManager {

    var unionFind = UnionFind()

    val islandManifold = ArrayList<PersistentManifold>()
    val islandBodies = ArrayList<CollisionObject>()

    var splitIslands = true

    fun initUnionFind(n: Int) = unionFind.reset(n)

    fun updateActivationState(colWorld: CollisionWorld, dispatcher: Dispatcher) {
        // put the index into controllers into tag
        var index = 0
        if (STATIC_SIMULATION_ISLAND_OPTIMIZATION) {
            // put the index into m_controllers into m_tag
            colWorld.collisionObjects.forEach {
                //Adding filtering here
                if (!it.isStaticOrKinematicObject)
                    it.islandTag = index++
                it.companionId = -1
                it.hitFraction = 1f
            }
            // do the union find
            initUnionFind(index)
            findUnions(dispatcher, colWorld)
        } else {
            initUnionFind(colWorld.collisionObjects.size)
            colWorld.collisionObjects.forEach {
                it.islandTag = index
                it.companionId = -1
                it.hitFraction = 1f
                index++
            }
            // do the union find
            findUnions(dispatcher, colWorld)
        }
    }

    fun storeIslandActivationState(world: CollisionWorld) {
        // put the islandId ('find' value) into tag
        var index = 0
        if (STATIC_SIMULATION_ISLAND_OPTIMIZATION)
            for (i in 0 until world.collisionObjects.size) {
                val collisionObject = world.collisionObjects[i]
                if (!collisionObject.isStaticOrKinematicObject) {
                    collisionObject.islandTag = unionFind.find(index)
                    //Set the correct object offset in Collision Object Array
                    unionFind[index].sz = i
                    collisionObject.companionId = -1
                    index++
                } else {
                    collisionObject.islandTag = -1
                    collisionObject.companionId = -2
                }
            }
        else
            for (i in 0 until world.collisionObjects.size) {
                val collisionObject = world.collisionObjects[i]
                if (!collisionObject.isStaticOrKinematicObject) {
                    collisionObject.islandTag = unionFind.find(index)
                    collisionObject.companionId = -1
                } else {
                    collisionObject.islandTag = -1
                    collisionObject.companionId = -2
                }
                index++
            }
    }

    fun findUnions(dispatcher: Dispatcher, colWorld: CollisionWorld) {
        val pairCachePtr = colWorld.pairCache
        val numOverlappingPairs = pairCachePtr.numOverlappingPairs
        if (numOverlappingPairs != 0) {
            val pairPtr = pairCachePtr.overlappingPairArray
            for (i in 0 until numOverlappingPairs) {
                val collisionPair = pairPtr[i]!!
                val colObj0 = collisionPair.proxy0!!.clientObject as? CollisionObject
                val colObj1 = collisionPair.proxy1!!.clientObject as? CollisionObject
                if (colObj0 != null && colObj0.mergesSimulationIslands() && colObj1 != null && colObj1.mergesSimulationIslands()) {
                    unionFind.unite(colObj0.islandTag, colObj1.islandTag)
                }
            }
        }
    }

    interface IslandCallback {
        fun processIsland(bodies: ArrayList<CollisionObject>, numBodies: Int, manifolds: ArrayList<PersistentManifold>,
                          manifoldsPtr: Int, numManifolds: Int, islandId: Int)
    }

    /** @todo: this is random access, it can be walked 'cache friendly'! */
    fun buildAndProcessIslands(dispatcher: Dispatcher, collisionWorld: CollisionWorld, callback: IslandCallback) {

        val collisionObjects = collisionWorld.collisionObjects
        buildIslands(dispatcher, collisionWorld)

        var endIslandIndex = 1
        var startIslandIndex = 0
        val numElem = unionFind.numElements
        BT_PROFILE("processIslands")
        if (!splitIslands) {
            val manifold = dispatcher.internalManifoldPointer!!
            val maxNumManifolds = dispatcher.numManifolds
            callback.processIsland(collisionObjects,  collisionObjects.size, manifold, 0, maxNumManifolds, -1)
        } else {
            /*  Sort manifolds, based on islands
                Sort the vector using predicate and std::sort
                std::sort(islandmanifold.begin(), islandmanifold.end(), btPersistentManifoldSortPredicate); */
            val numManifolds = islandManifold.size
            //tried a radix sort, but quicksort/heapsort seems still faster
            //@todo rewrite island management
            islandManifold.sortWith(PersistentManifoldSortPredicate)
            //now process all active islands (sets of manifolds for now)
            var startManifoldIndex = 0
            var endManifoldIndex = 1

            //	printf("Start Islands\n");
            //traverse the simulation islands, and call the solver, unless all objects are sleeping/deactivated
            startIslandIndex = 0
            while (startIslandIndex < numElem) {
                val islandId = unionFind[startIslandIndex].id
                var islandSleeping = true
                endIslandIndex = startIslandIndex
                while (endIslandIndex < numElem && unionFind[endIslandIndex].id == islandId) {
                    val i = unionFind[endIslandIndex].sz
                    val colObj0 = collisionObjects[i]
                    islandBodies += colObj0
                    if (colObj0.isActive) islandSleeping = false
                    endIslandIndex++
                }
                //find the accompanying contact manifold for this islandId
                var numIslandManifolds = 0
                var startManifold = 0
                if (startManifoldIndex < numManifolds) {
                    val curIslandId = islandManifold[startManifoldIndex].islandId
                    if (curIslandId == islandId) {
                        startManifold = startManifoldIndex
                        endManifoldIndex = startManifoldIndex + 1
                        while (endManifoldIndex < numManifolds && islandId == islandManifold[endManifoldIndex].islandId)
                            endManifoldIndex++
                        /// Process the actual simulation, only if not sleeping/deactivated
                        numIslandManifolds = endManifoldIndex - startManifoldIndex
                    }
                }
                if (!islandSleeping) {
                    callback.processIsland(islandBodies, islandBodies.size, islandManifold, startManifold, numIslandManifolds, islandId)
                    //			printf("Island callback of size:%d bodies, %d manifolds\n",islandBodies.size(),numIslandManifolds);
                }
                if (numIslandManifolds != 0) startManifoldIndex = endManifoldIndex
                islandBodies.clear()
                startIslandIndex = endIslandIndex
            }
        } // else if(!splitIslands)
    }

    fun buildIslands(dispatcher: Dispatcher, colWorld: CollisionWorld) {
        BT_PROFILE("islandUnionFindAndQuickSort")
        val collisionObjects = colWorld.collisionObjects
        islandManifold.clear()
        /*  we are going to sort the unionfind array, and store the element id in the size afterwards, we clean
            unionfind, to make sure no-one uses it anymore         */
        unionFind.sortIslands()
        val numElem = unionFind.numElements
        var endIslandIndex = 1
        var startIslandIndex = 0
        //update the sleeping state for bodies, if all are sleeping
        while (startIslandIndex < numElem) {
            val islandId = unionFind[startIslandIndex].id
            var endIslandIndex = startIslandIndex + 1
            while (endIslandIndex < numElem && unionFind[endIslandIndex].id == islandId)
                endIslandIndex++
            //int numSleeping = 0;
            var allSleeping = true
            for (idx in startIslandIndex until endIslandIndex) {
                val i = unionFind[idx].sz
                val colObj0 = collisionObjects[i]
                if (colObj0.islandTag != islandId && colObj0.islandTag != -1)
                    println("error in island management") // TODO logger?

                assert(colObj0.islandTag == islandId || colObj0.islandTag == -1)
                if (colObj0.islandTag == islandId)
                    if (colObj0.activationState == ACTIVE_TAG) allSleeping = false
                    else if (colObj0.activationState == DISABLE_DEACTIVATION) allSleeping = false
            }
            if (allSleeping)
                for (idx in startIslandIndex until endIslandIndex) {
                    val i = unionFind[idx].sz
                    val colObj0 = collisionObjects[i]
                    if (colObj0.islandTag != islandId && colObj0.islandTag != -1)
                        println("error in island management")
                    assert(colObj0.islandTag == islandId || colObj0.islandTag == -1)
                    if (colObj0.islandTag == islandId)
                        colObj0.activationState = ISLAND_SLEEPING
                }
            else
                for (idx in startIslandIndex until endIslandIndex) {
                    val i = unionFind[idx].sz
                    val colObj0 = collisionObjects[i]
                    if (colObj0.islandTag != islandId && colObj0.islandTag != -1)
                        println("error in island management")
                    assert(colObj0.islandTag == islandId || colObj0.islandTag == -1)
                    if (colObj0.islandTag == islandId && colObj0.activationState == ISLAND_SLEEPING) {
                        colObj0.activationState = WANTS_DEACTIVATION
                        colObj0.deactivationTime = 0f
                    }
                }
            startIslandIndex = endIslandIndex
        }
        val maxNumManifolds = dispatcher.numManifolds
        for (i in 0 until maxNumManifolds) {
            val manifold = dispatcher.getManifoldByIndexInternal(i)

            val colObj0 = manifold.body0
            val colObj1 = manifold.body1
            ///@todo: check sleeping conditions!
            if ((colObj0 != null && colObj0.activationState != ISLAND_SLEEPING) ||
                    (colObj1 != null && colObj1.activationState != ISLAND_SLEEPING)) {
                //kinematic objects don't merge islands, but wake up all connected objects
                if (colObj0!!.isKinematicObject && colObj0.activationState != ISLAND_SLEEPING)
                    if (colObj0.hasContactResponse) colObj1!!.activate()
                if (colObj1!!.isKinematicObject && colObj1.activationState != ISLAND_SLEEPING)
                    if (colObj1.hasContactResponse) colObj0.activate()
                if (splitIslands)
                //filtering for response
                    if (dispatcher.needsResponse(colObj0, colObj1))
                        islandManifold.add(manifold)
            }
        }
    }
}

private val PersistentManifold.islandId get() = if (body0!!.islandTag >= 0) body0!!.islandTag else body1!!.islandTag

object PersistentManifoldSortPredicate : Comparator<PersistentManifold> {
    override fun compare(o1: PersistentManifold, o2: PersistentManifold) = o1.islandId.compareTo(o2.islandId)
}
