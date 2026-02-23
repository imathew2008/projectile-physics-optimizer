/**
 * Numerically simulates projectile motion using explicit time stepping.
 *
 * At each step:
 *  1. Forces are computed via `computeForces`
 *  2. Acceleration is derived from F = ma
 *  3. Velocity and position are updated using forward Euler integration
 *  4. State is logged
 *
 * Simulation continues until `terminate` returns true.
 *
 * @param step simulation time step in seconds
 * @param initPosition initial position vector (meters)
 * @param initVelocity initial velocity vector (m/s)
 * @param mass projectile mass (kg)
 * @param computeForces function returning net force vector (N) given position and velocity
 * @param terminate function that determines when simulation stops
 * @param log callback invoked each step with time, position, velocity, and force
 */
inline fun projectileSim(
    step:          Double,
    initPosition:  Vector3,
    initVelocity:  Vector3,
    mass:          Double,
    computeForces: (pos: Vector3, vel: Vector3)                               -> Vector3,
    terminate:     (time: Double, pos: Vector3, vel: Vector3)                 -> Boolean,
    log:           (time: Double, pos: Vector3, vel: Vector3, force: Vector3) -> Unit,
) {
    var time     = 0.0
    var position = initPosition
    var velocity = initVelocity

    while (!terminate(time, position, velocity)) {
        val force        = computeForces(position, velocity)
        val acceleration = force / mass

        log(time, position, velocity, force)

        velocity += acceleration * step
        position += velocity * step
        time     += step
    }
}
/**
 * Generic iterative optimization framework over a discrete choice space.
 *
 * Starting from an initial parameter set `init`, this function:
 *  1. Generates candidate parameter mutations using `mutate`
 *  2. Simulates each candidate
 *  3. Selects the candidate with minimum `error`
 *  4. Repeats until `terminate` returns true
 *
 * Designed for tuning simulation parameters such as launch angle,
 * velocity, spin, or other control variables.
 *
 * @param P parameter type being optimized
 * @param Choice type representing mutation decisions
 * @param Results simulation result type
 *
 * @param init initial parameter set
 * @param choiceSpace iterable set of candidate mutation choices
 * @param mutate function that applies a choice to parameters (returns null if invalid)
 * @param simulate function that evaluates parameters and produces results
 * @param error function that scores simulation results (lower is better)
 * @param terminate function deciding when to stop iterating
 * @param log callback invoked each iteration with parameters and results
 *
 * @return best parameter set found at termination
 */
inline fun <P: Any, Choice, Results> optimize(
    init:                  P,
    choiceSpace:          Iterable<Choice>,
    crossinline mutate:   (P, Choice) -> P?,
    crossinline simulate: (P) -> Results,
    crossinline error:    (Results)                                -> Double,
    terminate:            (iters: Int, parms: P, result: Results)  -> Boolean,
    log      :            (iters: Int, parms: P, results: Results) -> Unit,
    ): P {

    var params  = init
    var iters   = 0
    val results = simulate(params)
    log(iters, params, results)

    do {
        val (p, res) = choiceSpace.mapNotNull { mutate(params, it) }
            .map { it to simulate(it) }
            .minByOrNull { error(it.second) }!!
        ++iters
        params = p
        log(iters, p, res)
    }while (!terminate(iters, p, res))

    return params
}
