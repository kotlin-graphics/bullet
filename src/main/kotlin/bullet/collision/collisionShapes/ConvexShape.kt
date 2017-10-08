package bullet.collision.collisionShapes

import bullet.linearMath.Vec3

abstract class ConvexShape : CollisionShape() {

    abstract fun localGetSupportingVertex(vec:Vec3):Vec3

    abstract fun localGetSupportingVertexWithoutMargin(vec:Vec3):Vec3

    abstract fun localGetSupportVertexWithoutMarginNonVirtual (vec:Vec3):Vec3
    abstract fun localGetSupportVertexNonVirtual (vec:Vec3):Vec3
//    btScalar getMarginNonVirtual () const;
//    void getAabbNonVirtual (const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const;
//
//
//    virtual void project(const btTransform& trans, const btVector3& dir, btScalar& minProj, btScalar& maxProj, btVector3& witnesPtMin,btVector3& witnesPtMax) const;
//
//
//    //notice that the vectors should be unit length
//    virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const= 0;
//
//    ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
//    void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const =0;
//
//    virtual void getAabbSlow(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const =0;
//
//    virtual void	setLocalScaling(const btVector3& scaling) =0;
//    virtual const btVector3& getLocalScaling() const =0;
//
//    virtual void	setMargin(btScalar margin)=0;
//
//    virtual btScalar	getMargin() const=0;
//
//    virtual int		getNumPreferredPenetrationDirections() const=0;
//
//    virtual void	getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const=0;
}