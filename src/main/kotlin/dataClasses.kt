import kotlin.math.sqrt

/**
 * 3D vector used for physics calculations.
 *
 * Assumes a right-handed coordinate system:
 *  +x = forward
 *  +y = up
 *  +z = out of plane (lateral)
 *
 * All units are expected to be consistent (e.g., meters for position,
 * meters/second for velocity, Newtons for force).
 */
data class Vector3(val x: Double, val y: Double, val z: Double) {
    operator fun plus (v: Vector3) = Vector3(x + v.x, y + v.y, z + v.z)
    operator fun minus(v: Vector3) = Vector3(x - v.x, y - v.y, z - v.z)
    operator fun times(s: Double)  = Vector3(x * s, y * s, z * s)
    operator fun div  (s: Double)  = Vector3(x / s, y / s, z / s)

    val norm: Double get() = sqrt(x * x + y * y + z * z)
   fun cross(v: Vector3) = Vector3(
        y * v.z - z * v.y,
        z * v.x - x * v.z,
        x * v.y - y * v.x)

    companion object {
        val zero = Vector3(0.0, 0.0, 0.0)
    }
}

/**
 * Snapshot of projectile state at a simulation step.
 *
 * @property forces net force applied during the step (N)
 * @property pos position in world coordinates (m)
 * @property vel velocity in world coordinates (m/s)
 */
data class ResultsToPrint(val forces: Vector3, val pos: Vector3, val vel: Vector3)

/**
 * Represents the estimated exit conditions of a ball leaving
 * a two-flywheel shooter.
 *
 * @property vExit linear exit speed (m/s)
 * @property omegaBall angular velocity about spin axis (rad/s)
 * @property ballRpm angular velocity converted to RPM
 */
data class BallExit(
    val vExit:     Double,
    val omegaBall: Double,
    val ballRpm:   Double)

/**
 * Results from a full projectile simulation.
 *
 * Includes geometric target metrics, entry angle error,
 * optimization cost, and sampled trajectory data.
 *
 * @property hitWindow true if trajectory enters target window
 * @property crossedPlane true if trajectory crosses target x-plane
 * @property cost scalar optimization score(lower is better)
 * @property pCross interpolated position at plane crossing(if any)
 * @property vCross interpolated velocity at plane crossing(if any)
 * @property pBest closest position to target
 * @property vBest velocity at closest approach
 * @property results full list of sampled simulation states
 * @property above true if projectile passes above window
 * @property downward true if entry velocity is downward
 * @property phiError angular entry error relative to goal angle
 * @property distance closest spatial error to target
 */
data class SimResult(
    val hitWindow:    Boolean,
    val crossedPlane: Boolean,
    val cost:         Double,
    val pCross:       Vector3?,
    val vCross:       Vector3?,
    val pBest:        Vector3,
    val vBest:        Vector3,
    val results:      List<ResultsToPrint>,
    val above:        Boolean,
    val downward:     Boolean,
    val phiError:     Double,
    val distance:     Double)