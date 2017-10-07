package bullet.collision.broadphaseCollision

/** The btDispatcher interface class can be used in combination with broadphase to dispatch calculations for
 *  overlapping pairs.
 *  For example for pairwise collision detection, calculating contact points stored in btPersistentManifold or user
 *  callbacks (game logic). */
class Dispatcher {

//    abstract btCollisionAlgorithm* findAlgorithm(const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap,btPersistentManifold* sharedManifold, ebtDispatcherQueryType queryType) = 0;
//
//    virtual btPersistentManifold*	getNewManifold(const btCollisionObject* b0,const btCollisionObject* b1)=0;
//
//    virtual void releaseManifold(btPersistentManifold* manifold)=0;
//
//    virtual void clearManifold(btPersistentManifold* manifold)=0;
//
//    virtual bool	needsCollision(const btCollisionObject* body0,const btCollisionObject* body1) = 0;
//
//    virtual bool	needsResponse(const btCollisionObject* body0,const btCollisionObject* body1)=0;
//
//    virtual void	dispatchAllCollisionPairs(btOverlappingPairCache* pairCache,const btDispatcherInfo& dispatchInfo,btDispatcher* dispatcher)  =0;
//
//    virtual int getNumManifolds() const = 0;
//
//    virtual btPersistentManifold* getManifoldByIndexInternal(int index) = 0;
//
//    virtual	btPersistentManifold**	getInternalManifoldPointer() = 0;
//
//    virtual	btPoolAllocator*	getInternalManifoldPool() = 0;
//
//    virtual	const btPoolAllocator*	getInternalManifoldPool() const = 0;
//
//    virtual	void* allocateCollisionAlgorithm(int size)  = 0;
//
//    virtual	void freeCollisionAlgorithm(void* ptr) = 0;
}