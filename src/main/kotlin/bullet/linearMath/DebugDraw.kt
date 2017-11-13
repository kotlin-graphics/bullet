package bullet.linearMath

class DebugDraw {

    var activeObject = Vec3(1f, 1f, 1f)
    var deactivatedObject = Vec3(0f, 1f, 0f)
    var wantsDeactivationObject = Vec3(0f, 1f, 1f)
    var disabledDeactivationObject = Vec3(1f, 0f, 0f)
    var disabledSimulationObject = Vec3(1f, 1f, 0f)
    var aabb = Vec3(1f, 0f, 0f)
    var contactPoint = Vec3(1f, 1f, 0f)

    enum class Modes(val i: Int) {
        NoDebug(0),
        DrawWireframe(1),
        DrawAabb(2),
        DrawFeaturesText(4),
        DrawContactPoints(8),
        NoDeactivation(16),
        NoHelpText(32),
        DrawText(64),
        ProfileTimings(128),
        EnableSatComparison(256),
        DisableBulletLCP(512),
        EnableCCD(1024),
        DrawConstraints(1 shl 11),
        DrawConstraintLimits(1 shl 12),
        FastWireframe(1 shl 13),
        DrawNormals(1 shl 14),
        DrawFrames(1 shl 15),
        MAX_DEBUG_DRAW_MODE(DrawFrames.i + 1)
    }

    fun drawSphere(c: Vec3, fl: Float, btVector3: Any) {}
}