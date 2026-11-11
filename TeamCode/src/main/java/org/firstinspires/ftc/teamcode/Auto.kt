/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */
package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.TranslationalVelConstraint
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.util.ElapsedTime

/**
 * Select RED/BLUE during INIT; start preloaded and aimed at that alliance's hive.
 * Four timed feeder pulses, then park using Road Runner.
 * FEED_SECONDS must be tested: without a ball sensor, pulses do not verify shots.
 */
@Autonomous(name = "Auto", group = "StarterBot")
class Auto : LinearOpMode() {
    private enum class TeamColor { RED, BLUE }
    private enum class StartPosition { HIVE, DELAYED }

    private var teamColor = TeamColor.RED
    private var startPosition = StartPosition.HIVE
    private var trustedMode = false
    private var delaySeconds = 10

    private fun selectedStart(): Pose2d {
        val pose = toSided(
            if (startPosition == StartPosition.HIVE) START_POSE else DELAYED_START_RED
        )
        // Trusted Mode means the robot is ALREADY physically facing the goal.
        // Initialize odometry to that orientation; do not command a 180-degree turn.
        return if (startPosition == StartPosition.DELAYED && trustedMode)
            Pose2d(pose.position, pose.heading.toDouble() + Math.PI)
        else pose
    }

    private fun runDriveAction(action: Action): Boolean {
        while (opModeIsActive()) {
            val packet = TelemetryPacket(false)
            BiobuzzField.apply(packet)
            val running = action.run(packet)
            FtcDashboard.getInstance().sendTelemetryPacket(packet)
            if (!running) return true
            idle()
        }
        return false
    }

    // BIOBUZZ's opposite-corner layout has 180-degree rotational symmetry.
    // Transform every waypoint and the starting heading together.
    private fun toSided(point: Vector2d): Vector2d =
        if (teamColor == TeamColor.RED) point else Vector2d(-point.x, -point.y)

    private fun toSided(pose: Pose2d): Pose2d =
        if (teamColor == TeamColor.RED) pose else Pose2d(
            toSided(pose.position),
            pose.heading.toDouble() + Math.PI
        )

    companion object {
        // Shooter settings match BobStarterbotTeleop.java (ticks per second).
        private const val LAUNCHER_TARGET_VELOCITY = 1250.0
        private const val LAUNCHER_MIN_VELOCITY = 1200.0
        private const val BALL_COUNT = 6 // we can carry 4 balls but we shooting 6 just in case
        private const val FEED_SECONDS = 0.35
        private const val RECOVERY_SECONDS = 0.45
        private const val SHOT_TIMEOUT_SECONDS = 4.0
        private const val PARK_MAX_VELOCITY = 20.0 // inches per second
        private const val FINAL_APPROACH_VELOCITY = 8.0
        private const val RED_TRAVEL_LANE_X = -46.0

        // Recorded red-side poses from the previous Auto.kt.
        private val START_POSE = Pose2d(-15.58, -63.32, Math.toRadians(-90.0))
        // Supplied BLUE pose: (-11.53, 64.05, -89 degrees).
        // Store its 180-degree counterpart so toSided() handles both alliances.
        private val DELAYED_START_RED = Pose2d(-13.44, 61.68, Math.toRadians(-90.8))
        private const val RED_TURN_CLEARANCE_Y = -48.0
        // Retain the team's latest park target. Center is 6 inches from the wall;
        // verify the robot footprint and localization error fit this clearance.
        private val PARK_POSITION = Vector2d(-66.0, 38.91 + 15.0)
    }

    override fun runOpMode() {
        val intake = hardwareMap.get(DcMotor::class.java, "intake")
        val launcher = hardwareMap.get(DcMotorEx::class.java, "launcher")
        val feeder = hardwareMap.get(CRServo::class.java, "windmillServo")

        // Explicitly set directions because earlier OpModes can change them.
        intake.direction = DcMotorSimple.Direction.FORWARD
        launcher.direction = DcMotorSimple.Direction.FORWARD
        feeder.direction = DcMotorSimple.Direction.FORWARD
        intake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        launcher.mode = DcMotor.RunMode.RUN_USING_ENCODER
        launcher.setPIDFCoefficients(
            DcMotor.RunMode.RUN_USING_ENCODER,
            PIDFCoefficients(40.0, 0.0, 0.0, 12.5)
        )
        intake.power = 0.0
        feeder.power = 0.0
        launcher.velocity = 0.0

        var drive: MecanumDrive? = null
        try {
            val robotDrive = MecanumDrive(hardwareMap, START_POSE)
            drive = robotDrive
            robotDrive.rightFront.direction = DcMotorSimple.Direction.FORWARD
            robotDrive.rightBack.direction = DcMotorSimple.Direction.FORWARD
            val driveMotors = listOf(
                robotDrive.leftFront, robotDrive.leftBack,
                robotDrive.rightFront, robotDrive.rightBack
            )
            driveMotors.forEach {
                it.power = 0.0
                it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }

            while (opModeInInit()) {
                if (gamepad1.xWasPressed()) teamColor = TeamColor.BLUE
                if (gamepad1.bWasPressed()) teamColor = TeamColor.RED
                if (gamepad1.aWasPressed()) {
                    startPosition = if (startPosition == StartPosition.HIVE)
                        StartPosition.DELAYED else StartPosition.HIVE
                    trustedMode = false
                }
                if (gamepad1.rightBumperWasPressed()) {
                    startPosition = StartPosition.DELAYED
                    trustedMode = false
                }
                if (gamepad1.leftBumperWasPressed()) {
                    startPosition = StartPosition.HIVE
                    trustedMode = false
                }
                if (gamepad1.yWasPressed() && startPosition == StartPosition.DELAYED)
                    trustedMode = !trustedMode
                if (gamepad1.dpadUpWasPressed()) delaySeconds = (delaySeconds + 1).coerceAtMost(20)
                if (gamepad1.dpadDownWasPressed()) delaySeconds = (delaySeconds - 1).coerceAtLeast(0)

                val selectedStart = selectedStart()
                val selectedPark = toSided(PARK_POSITION)
                telemetry.addLine("X = BLUE | B = RED")
                telemetry.addLine("A = HIVE / DELAYED start")
                telemetry.addLine("RB = select DELAYED | LB = select HIVE (Trusted resets OFF)")
                telemetry.addLine("Y = Trusted Mode (already aimed; flips starting heading 180 deg)")
                telemetry.addLine("D-pad up/down = delay +/- 1 second (0-20)")
                telemetry.addData("Alliance", teamColor)
                telemetry.addData("Starting position", startPosition)
                telemetry.addData("Delay (delayed start only)", delaySeconds)
                telemetry.addData("Trusted Mode (delayed start only)", trustedMode)
                telemetry.addData("Shooting plan", when {
                    startPosition == StartPosition.HIVE -> "SHOOT NOW at hive, then PARK"
                    trustedMode -> "WAIT ${delaySeconds}s, SHOOT HERE, then PARK"
                    else -> "WAIT ${delaySeconds}s, DRIVE TO HIVE, SHOOT, then PARK"
                })
                telemetry.addData("Start (in)", "x %.2f, y %.2f",
                    selectedStart.position.x, selectedStart.position.y)
                telemetry.addData("Start heading (deg)",
                    Math.toDegrees(selectedStart.heading.toDouble()))
                telemetry.addData("Park (in)", "x %.2f, y %.2f",
                    selectedPark.x, selectedPark.y)
                telemetry.addLine("Place robot at the displayed starting position AND heading.")
                telemetry.addLine("Four feed pulses, inset travel lane, then slow park approach.")
                telemetry.update()
                sleep(20)
            }
            waitForStart()
            if (isStopRequested) return

            val startPose = selectedStart()
            robotDrive.localizer.setPose(startPose)

            if (startPosition == StartPosition.DELAYED) {
                val delay = ElapsedTime()
                while (opModeIsActive() && delay.seconds() < delaySeconds) {
                    robotDrive.updatePoseEstimate()
                    telemetry.addData("Status", "Waiting")
                    telemetry.addData("Next", if (trustedMode) "Shoot here" else "Drive to hive")
                    telemetry.addData("Seconds remaining", "%.1f", delaySeconds - delay.seconds())
                    telemetry.update()
                    sleep(20)
                }
                if (!opModeIsActive()) return

                if (!trustedMode) {
                    telemetry.addData("Status", "Driving to hive; shooter OFF")
                    telemetry.update()
                    val currentPose = robotDrive.localizer.getPose()
                    val hivePose = toSided(START_POSE)
                    // Pull away from the wall before rotating, then approach hive.
                    val approach = robotDrive.actionBuilder(currentPose)
                        .strafeTo(Vector2d(currentPose.position.x, hivePose.position.y), TranslationalVelConstraint(PARK_MAX_VELOCITY))
                        .strafeTo(hivePose.position, TranslationalVelConstraint(PARK_MAX_VELOCITY))
                        .build()
                    runBlocking(
                        approach
                    )
                    // A time-finished trajectory is not proof the robot arrived.
                    // Do not feed balls at the delayed start if the drive failed.
                    robotDrive.updatePoseEstimate()
                    val actual = robotDrive.localizer.getPose()
                    val positionError = Math.hypot(
                        actual.position.x - hivePose.position.x,
                        actual.position.y - hivePose.position.y
                    )
                    val headingDelta = actual.heading.toDouble() - hivePose.heading.toDouble()
                    val headingError = Math.abs(Math.atan2(Math.sin(headingDelta), Math.cos(headingDelta)))
                    if (positionError > 3.0 || headingError > Math.toRadians(15.0)) {
                        telemetry.addData("Status", "Hive not reached: shooting cancelled")
                        telemetry.addData("Position error (in)", positionError)
                        telemetry.addData("Heading error (deg)", Math.toDegrees(headingError))
                        telemetry.update()
                        while (opModeIsActive()) sleep(50)
                        return
                    }
                }
            }

            // Stay inward of the wall flowers until past them, then approach park.
            // These waypoints need a physical clearance check for the robot footprint.
            robotDrive.updatePoseEstimate()
            val shootingPose = robotDrive.localizer.getPose()
            val upShootingPose = Pose2d(Vector2d(shootingPose.position.x, shootingPose.position.y + if (teamColor == TeamColor.RED) -25 else 25), shootingPose.heading)
            val parkFoward = robotDrive.actionBuilder(shootingPose).strafeTo(upShootingPose.position).build()
            val park = robotDrive.actionBuilder(if (trustedMode) upShootingPose else shootingPose)
                .strafeTo(
                    Vector2d(toSided(Vector2d(RED_TRAVEL_LANE_X, 0.0)).x,
                        shootingPose.position.y),
                    TranslationalVelConstraint(PARK_MAX_VELOCITY)
                )
                .strafeTo(
                    toSided(Vector2d(RED_TRAVEL_LANE_X, PARK_POSITION.y)),
                    TranslationalVelConstraint(PARK_MAX_VELOCITY)
                )
                .strafeTo(toSided(PARK_POSITION),
                    TranslationalVelConstraint(FINAL_APPROACH_VELOCITY))
                .build()

            if (!opModeIsActive()) return
            telemetry.addData("Status", "Shooting")
            telemetry.update()
            launcher.velocity = LAUNCHER_TARGET_VELOCITY
            for (shot in 1..BALL_COUNT) {
                if (!opModeIsActive()) return

                val timeout = ElapsedTime()
                val interval = ElapsedTime()
                var feedTime = 0.0
                var wasFeeding = false
                while (opModeIsActive() && feedTime < FEED_SECONDS &&
                    timeout.seconds() < SHOT_TIMEOUT_SECONDS
                ) {
                    // Count only time the feeder was powered; pause on speed dips.
                    val dt = interval.seconds()
                    interval.reset()
                    if (wasFeeding) feedTime += dt
                    val ready = launcher.velocity >= LAUNCHER_MIN_VELOCITY &&
                        feedTime < FEED_SECONDS
                    feeder.power = if (ready) 1.0 else 0.0
                    intake.power = if (ready) 0.5 else 0.0
                    wasFeeding = ready
                    telemetry.addData("Feed pulse", "%d / %d", shot, BALL_COUNT)
                    telemetry.addData("Launcher ticks/sec", launcher.velocity)
                    telemetry.update()
                    sleep(10)
                }
                feeder.power = 0.0
                intake.power = 0.0
                if (!opModeIsActive()) return
                if (feedTime < FEED_SECONDS) {
                    telemetry.addLine("Shooter timeout: skipping remaining pulses and parking.")
                    telemetry.update()
                    break
                }
                sleep((RECOVERY_SECONDS * 1000).toLong())
            }

            launcher.velocity = 0.0
            feeder.power = 0.0
            intake.power = 0.0
            telemetry.addData("Status", "Parking")
            telemetry.update()

            if (trustedMode) {
                runDriveAction(parkFoward)
            }
            runDriveAction(park)
        } finally {
            feeder.power = 0.0
            intake.power = 0.0
            launcher.velocity = 0.0
            launcher.power = 0.0
            drive?.let {
                it.leftFront.power = 0.0
                it.leftBack.power = 0.0
                it.rightFront.power = 0.0
                it.rightBack.power = 0.0
            }
        }
    }
}
