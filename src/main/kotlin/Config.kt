import kotlin.math.*

val config = SimConfig(
    environment = Environment(
        airDensity  = 1.225,
        dragCoeff   = 0.4,
        magnusCoeff = 0.15,
        gravity     = -9.81
    ),
    projectile  = Projectile(
        mass   = 0.45,
        radius = 0.0751
    ),
    simulation  = Simulation(
        dt       = 0.01,
        maxTime  = 100.0,
        maxSpeed = 80.0,
        floor    = 0.0,
        dir      = (0..31).map { it/32.0 * 2 * PI }
            .map{ Vector3(cos(it), sin(it), 0.0) } + Vector3.zero
    ),
    target      = Target(
        goal      = Vector3(3.05, 1.8, 0.0),
        goalAngle = Math.toRadians(-45.0),
        yMin      = 1.79,
        yMax      = 1.81
    ),
    shooter    = Shooter(
        topRadi    = 0.1016 / 2.0,
        bottomRadi = 0.0508 / 2.0,
        robot0     = Vector3.zero,
        robotv0    = Vector3.zero,
        hoodAngle0 = 60.0,
        topRpm     = 3000.0,
        bottomRpm  = 3000.0
    )
)