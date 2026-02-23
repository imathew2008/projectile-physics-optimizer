import kotlin.math.*

val exitSpecs = ballExitFromTwoWheels(
    topWheelRadius    = config.shooter.topRadi,
    bottomWheelRadius = config.shooter.bottomRadi,
    topRpm            = config.shooter.topRpm,
    bottomRpm         = config.shooter.bottomRpm,
    ballRadius        = config.projectile.radius,
    vScale            = 0.85,
    spinScale         = 0.7)

val v0ball  = Vector3(cos(config.shooter.hoodAngle0) * exitSpecs.vExit, sin(config.shooter.hoodAngle0)
        * exitSpecs.vExit, 0.0) + config.shooter.robotv0
val omega   = Vector3(0.0, 0.0, exitSpecs.omegaBall)
val gravity = Vector3(0.0, config.environment.gravity * config.projectile.mass, 0.0)

/**
 * Runs a simple forward Euler simulation to generate an initial trajectory guess.
 *
 * This function is primarily used for visualization and baseline estimation
 * before running the full optimization pipeline.
 *
 * Physics options:
 *  - gravity only
 *  - gravity + t
 *  - gravity + drag + Magnus (spin)
 *
 * Simulation stops when:
 *  - time exceeds maxTime
 *  - projectile falls below ground (y < 0)
 *
 * @param p0 initial position (meters)
 * @param v0 initial velocity (m/s)
 * @param dt time step (seconds)
 * @param airResistance whether quadratic drag is included
 * @param spin whether Magnus force is included
 *
 * @return list of sampled states (force, position, velocity) over time
 */
fun initialGuess(p0: Vector3, v0: Vector3, dt: Double, airResistance: Boolean, spin: Boolean, ): List<ResultsToPrint> {
    println("gravity=${config.environment.gravity}, mass=${config.projectile.mass}, dt=${config.simulation.dt}")
    var p = p0
        var v = v0
        var t = 0.0
        val results = mutableListOf<ResultsToPrint>()

        while(t <= config.simulation.maxTime && p.y >= 0.0) {
            val force = if(airResistance) {
                if(spin) gravity + dragForce(v) + magnusForce(v, omega) else gravity + dragForce(v)
            } else gravity
            val a = force / config.projectile.mass
            v += a * dt
            p += v * dt
            t += dt

            results.add(ResultsToPrint(force, p, v))
        }
        return results
    }
/**
 * Simulates a projectile trajectory and evaluates its performance relative to a target.
 *
 * Internally uses `projectileSim` for time integration and tracks:
 *  - Closest approach to the goal
 *  - Whether the trajectory crosses the goal x-plane
 *  - Best interpolation point at plane crossing
 *  - Entry window hit detection
 *  - Downward entry angle validation
 *
 * The returned SimResult includes geometric errors, angle error,
 * and a combined scalar cost used for optimization.
 *
 * Termination conditions:
 *  - time > maxTime
 *  - y position below floor
 *  - speed exceeds maxSpeed
 *
 * @param p0 initial position (meters)
 * @param v0 initial velocity (m/s)
 * @param dt time step (seconds)
 * @param airResistance include quadratic drag
 * @param spin include Magnus force
 *
 * @return SimResult containing trajectory metrics and samples
 */
fun sim(p0: Vector3, v0: Vector3, dt: Double, airResistance: Boolean, spin: Boolean): SimResult {

    var pBest            = p0
    var vBest            = v0
    var pPrev            = p0
    var vPrev            = v0
    var crossedPlane     = false
    var hitWindow        = false
    var above            = false
    var downward         = false
    var pCross: Vector3? = null
    var vCross: Vector3? = null
    var bestPlaneDist    = Double.POSITIVE_INFINITY
    var bestCrossErr     = Double.POSITIVE_INFINITY

    val samples = mutableListOf<ResultsToPrint>()

    projectileSim(
        step          = dt,
        initPosition  = p0,
        initVelocity  = v0,
        mass          = config.projectile.mass,
        computeForces =
            { _, vel ->
                if(airResistance && spin) {
                gravity + dragForce(vel) + magnusForce(vel, omega)
            } else if (airResistance) gravity + dragForce(vel) else gravity}
        ,
        terminate     = { time, pos, vel ->
            time > config.simulation.maxTime || pos.y < config.simulation.floor || vel.norm > config.simulation.maxSpeed
        },
        log           = { _, pos, vel, force ->
            val dist = (pos - config.target.goal).norm
            if(dist < bestPlaneDist) {
                bestPlaneDist = dist
                pBest         = pos
                vBest         = vel
            }

            val a = crossXPlane(pPrev, pos, config.target.goal.x)
            if (a != null) {
                crossedPlane = true
                val pAt = pPrev + (pos - pPrev) * a
                val vAt = vPrev + (vel - vPrev) * a

                val yzErr = hypot(pAt.y - config.target.goal.y, pAt.z - config.target.goal.z)
                if (yzErr < bestCrossErr) {
                    bestCrossErr = yzErr
                    pCross       = pAt
                    vCross       = vAt
                }
                if (pAt.y in config.target.yMin..config.target.yMax) hitWindow = true
                if (pAt.y > config.target.yMax) above = true
                if (angle(vAt) < 0.0) downward = true
            }

            pPrev    = pos
            vPrev    = vel
            samples += ResultsToPrint(force, pos, vel)
        }
    )
    val posErr = if (pCross != null) bestCrossErr else bestPlaneDist
    val phiErr = abs(wrapAngle(angle(vCross ?: vBest) - config.target.goalAngle))
    val cost   = posErr + phiErr * 0.8

    return SimResult(
        hitWindow    = hitWindow,
        crossedPlane = crossedPlane,
        cost         = cost,
        pCross       = pCross,
        vCross       = vCross,
        pBest        = pBest,
        vBest        = vBest,
        results      = samples,
        above        = above,
        downward     = downward,
        phiError     = phiErr,
        distance     = posErr
    )
}
/**
 * Performs iterative discrete optimization to find an initial velocity
 * that minimizes trajectory cost.
 *
 * Uses `optimize` with directional perturbations to adjust velocity.
 * Step size (factor) is adaptively reduced when no improvement occurs.
 *
 * Optimization continues until perturbation factor becomes sufficiently small.
 *
 * @param spin include Magnus force during simulation
 * @param airResistance include quadratic drag during simulation
 * @param log optional per-iteration callback for debugging or visualization
 *
 * @return Pair of:
 *  - best velocity vector found
 *  - corresponding SimResult
 */
inline fun speedOptimizer(spin: Boolean, airResistance: Boolean, log: (Int, Vector3, SimResult) -> Unit =
    { _, _, _ -> }): Pair<Vector3, SimResult> {

    val seed    = v0ball
    var bestV   = seed
    var bestRes = sim(config.shooter.robot0, bestV, config.simulation.dt, airResistance, spin)
    var factor  = 2.0

    optimize(
        init        = bestV,
        choiceSpace = config.simulation.dir,
        mutate      = { current, dir ->
            val cand = current + dir * factor
            if (!possibleVelocity(cand)) null else cand
        },
        simulate    = { v -> sim(config.shooter.robot0, v, config.simulation.dt, airResistance, spin) },
        error       = { res -> res.cost },
        terminate   = { _, _, res ->
            if(res.cost < bestRes.cost) {
                bestRes = res
                bestV   = res.vBest
            } else {
                factor /= 2.0
            }

            factor < 1/1024.0
        },
        log        = { a, b, c -> log(a, b, c) }
    )
    return bestV to bestRes
}