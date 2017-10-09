package bullet.linearMath


fun transformAabb(localAabbMin: Vec3, localAabbMax: Vec3, margin: Float, trans: Transform, aabbMinOut: Vec3, aabbMaxOut: Vec3) {
    assert(localAabbMin.x <= localAabbMax.x)
    assert(localAabbMin.y <= localAabbMax.y)
    assert(localAabbMin.z <= localAabbMax.z)
    val localHalfExtents = 0.5f * (localAabbMax - localAabbMin)
    localHalfExtents += Vec3(margin)

    val localCenter = 0.5f * (localAabbMax + localAabbMin)
    val absB = trans.basis.absolute()
    val center = trans * localCenter
    val extent = localHalfExtents.dot3(absB[0], absB[1], absB[2])
    aabbMinOut put (center - extent)
    aabbMaxOut put (center + extent)
}