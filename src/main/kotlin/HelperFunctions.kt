import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.DrawScope
import kotlin.math.atan2

/**
 * Returns the angle of a vector in radians
 * @param v vector
 */
fun angle(v: Vector3) = atan2(v.y, v.x)
/**
 * Convert rpm to radians per second
 * @param rpm rpm to convert
 * @return radians per second
 */
fun rpmToRadPerSec(rpm: Double) = rpm * 2.0 * Math.PI / 60.0
/**
 * Converts radians per second to rpm
 * @param w radians per second to convert
 * @return rpm
 */
fun radPerSecToRpm(w: Double)   = w * 60.0 / (2.0 * Math.PI)
/**
 * Wraps angle(radians) to (-π, π]
 * @param a0 angle to wrap
 * @return angle between -π and π
 */
fun wrapAngle(a0: Double): Double {
    var a = a0
    while (a <= -Math.PI) a += 2.0 * Math.PI
    while (a > Math.PI) a -= 2.0 * Math.PI
    return a
}
/**
 * Computes wheel surface(tangential) speed in m/s from wheel diameter and RPM.
 * @param diameterM wheel diameter in meters
 * @param wheelRpm wheel speed in RPM
 * @return surface speed in m/s
 */
fun wheelSurfaceSpeed(diameterM: Double, wheelRpm: Double): Double {
    val r = diameterM * 0.5
    val w = rpmToRadPerSec(wheelRpm)
    return w * r
}
/**
 * Estimates the ball's exit linear velocity and spin from a two-roller shooter.
 *
 * Uses average wheel surface speed for exit velocity and wheel speed difference
 * to estimate ball angular velocity (spin) about the shooter spin axis.
 *
 * @param topWheelRadius radius of the top wheel (meters)
 * @param bottomWheelRadius radius of the bottom wheel (meters)
 * @param topRpm top wheel RPM
 * @param bottomRpm bottom wheel RPM
 * @param ballRadius ball radius (meters)
 * @param vScale empirical scaling factor for exit speed (defaults to 1.0)
 * @param spinScale empirical scaling factor for spin (defaults to 1.0)
 * @return BallExit containing exit speed (m/s), spin (rad/s), and spin in RPM
 */
fun ballExitFromTwoWheels(
    topWheelRadius:    Double,
    bottomWheelRadius: Double,
    topRpm:            Double,
    bottomRpm:         Double,
    ballRadius:        Double,
    vScale:            Double,
    spinScale:         Double
): BallExit {

    val uTop = wheelSurfaceSpeed(topWheelRadius, topRpm)
    val uBot = wheelSurfaceSpeed(bottomWheelRadius, bottomRpm)
    val vExit = vScale * 0.5 * (uTop + uBot)
    val omegaBall = spinScale * (uTop - uBot) / (2.0 * ballRadius)
    val ballRpm = radPerSecToRpm(omegaBall)

    return BallExit(vExit, omegaBall, ballRpm)
}
/**
 * Computes the Magnus (lift) force on a spinning projectile.
 *
 * Direction is given by ω × v. Magnitude is modeled using a lift coefficient
 * proportional to the spin ratio (R|ω|/|v|).
 *
 * Note: This assumes speed > 0 and ω × v is non-zero; callers should guard
 * against division by zero if needed.
 *
 * @param v projectile velocity vector (m/s)
 * @param omega projectile angular velocity vector (rad/s)
 * @return Magnus force vector (N)
 */
fun magnusForce(v: Vector3, omega: Vector3): Vector3 {
    val speed = v.norm
    val dir = omega.cross(v)
    val dirNorm = dir.norm
    val spinRatio = config.projectile.radius * omega.norm / speed
    val cl = config.environment.magnusCoeff * spinRatio
    val liftMag = 0.5 * config.environment.airDensity * speed * speed * config.projectile.area * cl

    return (dir / dirNorm) * liftMag
}
/**
 * Computes aerodynamic drag force opposing the direction of motion.
 *
 * Uses a quadratic drag model: Fd = -0.5 * rho * Cd * A * |v| * v
 *
 * @param v projectile velocity vector (m/s)
 * @return drag force vector (N)
 */
fun dragForce(v: Vector3): Vector3 {
    val k = -0.5 * config.environment.dragCoeff * config.environment.airDensity * config.projectile.area
    return v * (k * v.norm)
}
/**
 * Computes the interpolation fraction 'a' where a segment crosses the plane x = xGoal.
 *
 * pCross = pPrev + a*(p - pPrev), where 0 <= a <= 1.
 *
 * @param pPrev previous position
 * @param p current position
 * @param xGoal x-plane to test against
 * @return interpolation fraction a in [0, 1], or null if no crossing / undefined
 */
fun crossXPlane(pPrev: Vector3, p: Vector3, xGoal: Double): Double? {
    val dx = p.x - pPrev.x
    if (dx == 0.0) return null
    val a = (xGoal - pPrev.x) / dx
    return if (a in 0.0..1.0) a else null
}
/**
 * Checks whether a candidate velocity is physically/algorithmically acceptable.
 *
 * Current policy:
 * - Must have positive x component (forward)
 * - Speed must be <= maxSpeed
 *
 * @param v candidate velocity vector
 * @return true if allowed, false otherwise
 */
fun possibleVelocity(v: Vector3): Boolean {
    if (v.x <= 0.0) return false
    if (v.norm > config.simulation.maxSpeed) return false
    return true
}
/**
 * Projects a 3D world position into a centered top-down XZ inset view.
 *
 * x is mapped to screen +x, z is mapped to screen -y (so +z appears upward).
 * Centering is done relative to (goalX, goalZ).
 *
 * @param p world position
 * @param goalX x center reference (world units)
 * @param goalZ z center reference (world units)
 * @param origin screen-space origin of the inset
 * @param scale pixels per world unit
 * @return screen-space Offset for drawing
 */
fun topDownProjectXZCentered(
    p:      Vector3,
    goalX:  Double,
    goalZ:  Double,
    origin: Offset,
    scale:  Float
): Offset {
    val dx = (p.x - goalX).toFloat()
    val dz = (p.z - goalZ).toFloat()

    return Offset(
        origin.x + dx * scale,
        origin.y - dz * scale
    )
}
/**
 * Draws a thick line segment representing part of a trajectory.
 * @param color line color
 * @param start start point in screen space
 * @param end end point in screen space
 */
fun DrawScope.trajectoryLine(color: Color, start: Offset, end: Offset) {
    drawLine(color = color, start = start, end = end, strokeWidth = 4f)
}
/**
 * Converts world XY coordinates into screen space for a 2D side view.
 *
 * x is mapped to screen +x, y is mapped to screen -y (so +y appears upward).
 *
 * @param p world position
 * @param origin screen-space origin
 * @param scale pixels per world unit
 * @return screen-space Offset for drawing
 */
fun worldToScreen(p: Vector3, origin: Offset, scale: Float): Offset =
    Offset(origin.x + p.x.toFloat() * scale, origin.y - p.y.toFloat() * scale)
/**
 * Convenience wrapper for projecting into the top-down inset view using a shared goal.x.
 *
 * @param p world position
 * @param goalZ z center reference (world units)
 * @param insetCenter screen-space origin of the inset
 * @param scaleInset pixels per world unit (inset)
 * @return screen-space Offset for drawing in the inset
 */
fun projTop(p: Vector3, goalZ: Double, insetCenter: Offset, scaleInset: Float): Offset =
    topDownProjectXZCentered(p, config.target.goal.x, goalZ, insetCenter, scaleInset)
/**
 * Draws a connected polyline trajectory from a list of logged simulation results.
 *
 * If the list is null or has fewer than 2 points, nothing is drawn.
 *
 * @param list list of results containing positions to draw
 * @param color line color
 * @param origin screen-space origin for projection
 * @param scale pixels per world unit
 */
fun DrawScope.drawTrajectory(
    list:   List<ResultsToPrint>?,
    color:  Color,
    origin: Offset,
    scale:  Float
) {
    list?.let {
        if (it.size < 2) return
        for (i in 1 until it.size) {
            val a = worldToScreen(it[i - 1].pos, origin, scale)
            val b = worldToScreen(it[i].pos, origin, scale)
            trajectoryLine(color, a, b)
        }
    }
}
