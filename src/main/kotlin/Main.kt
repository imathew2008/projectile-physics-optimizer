import kotlin.math.*
import androidx.compose.ui.window.*
import androidx.compose.runtime.*
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material.MaterialTheme
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import kotlinx.coroutines.delay

/**
 * Projectile trajectory visualization UI(Compose).
 *
 * Runs the velocity optimizer under multiple physics models and plots:
 *  - Initial seed guess trajectories
 *  - Optimized trajectories
 *  - Target window on the goal x-plane
 *  - Top-down XZ window in the top right corner
 *
 * Coordinate system:
 *  +x = forward, +y = up, +z = out-of-plane
 *
 * Notes:
 *  - Trajectories are drawn in the x–y plane, with a smaller window rendering x–z.
 */

// Trajectory color legend:
//   Gray    = initial guess(gravity only)
//   Red     = initial guess + drag
//   Magenta = initial guess + drag + Magnus(spin)
//   Green   = optimized result(gravity only)
//   Blue    = optimized result + drag
//   Cyan    = optimized result + drag + Magnus(spin)

fun main() = application {

    var bestRes       by remember { mutableStateOf<SimResult?>(null) }
    var bestResAR     by remember { mutableStateOf<SimResult?>(null) }
    var bestResSpinAR by remember { mutableStateOf<SimResult?>(null) }

    LaunchedEffect(Unit) {
        speedOptimizer(spin = false, airResistance = false) { _, _, result ->
            bestRes = result
            delay(250)
        }
        println("Speed(m/s): ${bestRes!!.results[0].vel.norm}")
        println("Angle(degrees): ${Math.toDegrees(angle((bestRes!!.results[0].vel)))}")
    }

    LaunchedEffect(Unit) {
        speedOptimizer(spin = false, airResistance = true) { _, _, result ->
            bestResAR = result
            delay(250)
        }
        println("Speed(m/s) with air resistance: ${bestResAR!!.results[0].vel.norm}")
        println("Angle(degrees) with air resistance: ${Math.toDegrees(angle((bestResAR!!.results[0].vel)))}\n")
    }

    LaunchedEffect(Unit) {
        speedOptimizer(spin = true, airResistance = true) { _, _, result ->
            bestResSpinAR = result
            delay(250)
        }
        println("\nSpeed(m/s) with spin: ${bestResSpinAR!!.results[0].vel.norm}")
        println("Angle(degrees) with spin: ${Math.toDegrees(angle((bestResSpinAR!!.results[0].vel)))}\n")
    }

    Window(onCloseRequest = ::exitApplication, title = "Projectile Sim") {
        MaterialTheme {
            Canvas(Modifier.fillMaxSize()) bail@{
                val results         = bestRes      ?.results ?: return@bail
                val resultsAR       = bestResAR    ?.results ?: return@bail
                val resultsSpinAR   = bestResSpinAR?.results ?: return@bail
                val initGuessAR     = initialGuess(p0, v0ball, dt, airResistance = true,  spin = false)
                val initGuess       = initialGuess(p0, v0ball, dt, airResistance = false, spin = false)
                val initGuessSpinAR = initialGuess(p0, v0ball, dt, airResistance = true,  spin = true)

                val all = buildList {
                    addAll(initGuess)
                    addAll(initGuessAR)
                    addAll(initGuessSpinAR)
                    addAll(results)
                    addAll(resultsAR)
                    addAll(resultsSpinAR)
                }

                val minX = all.minOf { it.pos.x }
                val maxX = all.maxOf { it.pos.x }
                val minY = all.minOf { it.pos.y }
                val maxY = all.maxOf { it.pos.y }

                val padding      = 40f
                val insetPadding = 16f
                val insetMargin  = 12f
                val goalZ        = 0.0
                var maxAbsDx     = 0.1
                var maxAbsDz     = 0.1

                val worldWidth     = (maxX - minX).coerceAtLeast(1e-6)
                val worldHeight    = (maxY - minY).coerceAtLeast(1e-6)
                val drawableWidth  = size.width  - 2 * padding
                val drawableHeight = size.height - 2 * padding


                val scaleX = drawableWidth  / worldWidth.toFloat()
                val scaleY = drawableHeight / worldHeight.toFloat()
                val scale  = min(scaleX, scaleY)
                val origin = Offset(padding, size.height - padding)

                val goalBottom = worldToScreen(Vector3(goal.x, yMin - 0.127, 0.0), origin, scale)
                val goalTop    = worldToScreen(Vector3(goal.x, yMax + 0.127, 0.0), origin, scale)

                val insetSize     = min(size.width, size.height) * 0.35f
                val insetTopLeft  = Offset(size.width - insetSize - insetPadding, insetPadding)
                val insetCenter   = Offset(insetTopLeft.x + insetSize / 2f, insetTopLeft.y + insetSize / 2f)
                val insetDrawable = insetSize - 2f * insetMargin

                drawTrajectory(initGuessSpinAR, Color.Magenta, origin, scale)
                drawTrajectory(initGuess,       Color.Gray,    origin, scale)
                drawTrajectory(initGuessAR,     Color.Red,     origin, scale)
                drawTrajectory(results,         Color.Green,   origin, scale)
                drawTrajectory(resultsAR,       Color.Blue,    origin, scale)
                drawTrajectory(resultsSpinAR,   Color.Cyan,    origin, scale)

                trajectoryLine(Color.Black, goalBottom, goalTop)

                drawLine(Color.Black, Offset(insetCenter.x - 10, insetCenter.y),
                    Offset(insetCenter.x + 10, insetCenter.y), 2f)

                drawLine(Color.Black, Offset(insetCenter.x, insetCenter.y - 10),
                    Offset(insetCenter.x, insetCenter.y + 10), 2f)

                drawRect(
                    color   = Color(0x22FFFFFF),
                    topLeft = insetTopLeft,
                    size    = androidx.compose.ui.geometry.Size(insetSize, insetSize)
                )
                drawRect(
                    color   = Color.Black,
                    topLeft = insetTopLeft,
                    size    = androidx.compose.ui.geometry.Size(insetSize, insetSize),
                    style   = androidx.compose.ui.graphics.drawscope.Stroke(width = 2f)
                )

                val topDownLists: List<Pair<List<ResultsToPrint>, Color>> = buildList {
                    add(initGuessSpinAR to Color.Magenta)
                    add(initGuess       to Color.Gray)
                    add(initGuessAR     to Color.Red)
                    add(results         to Color.Green)
                    add(resultsAR       to Color.Blue)
                    add(resultsSpinAR   to Color.Cyan)
                }

                for ((list, _) in topDownLists) {
                    for (r in list) {
                        val dx = abs(r.pos.x - goal.x)
                        val dz = abs(r.pos.z - goalZ)
                        if (dx > maxAbsDx) maxAbsDx = dx
                        if (dz > maxAbsDz) maxAbsDz = dz
                    }
                }

                val halfSpan      = max(maxAbsDx, maxAbsDz).coerceAtLeast(1e-6)
                val scaleInset    = (insetDrawable / (2.0 * halfSpan)).toFloat()

                for ((list, color) in topDownLists) {
                    for (i in 1 until list.size) {
                        val a = projTop(list[i - 1].pos, goalZ, insetCenter, scaleInset)
                        val b = projTop(list[i].pos,     goalZ, insetCenter, scaleInset)

                        val minX = insetTopLeft.x
                        val maxX = insetTopLeft.x + insetSize
                        val minY = insetTopLeft.y
                        val maxY = insetTopLeft.y + insetSize
                        if ((a.x < minX - 50 && b.x < minX - 50) || (a.x > maxX + 50 && b.x > maxX + 50)) continue
                        if ((a.y < minY - 50 && b.y < minY - 50) || (a.y > maxY + 50 && b.y > maxY + 50)) continue

                        drawLine(
                            color       = color,
                            start       = a,
                            end         = b,
                            strokeWidth = 3f
                        )
                    }
                }
            }
        }
    }
}