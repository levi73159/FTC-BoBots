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

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.abs
import kotlin.math.max

/**
 * Driver controls for the BIOBUZZ goBILDA StarterBot:
 * left stick drives/strafes, right stick turns, triggers control intake,
 * and right bumper spins up the launcher and feeds when it is ready.
 */
@TeleOp(name = "Bob StarterBot Teleop", group = "StarterBot")
class BobStarterbotTeleop : OpMode() {
    private lateinit var leftFrontDrive: DcMotor
    private lateinit var leftBackDrive: DcMotor
    private lateinit var rightFrontDrive: DcMotor
    private lateinit var rightBackDrive: DcMotor
    private lateinit var launcher: DcMotorEx
    private lateinit var intake: DcMotor
    private lateinit var windmillServo: CRServo

    // Arm work in progress from the Java source; no hardware/control was defined.
    private lateinit var arm0: Servo
    private lateinit var arm1: Servo
    // Original unfinished declaration: private Se
    // private lateinit var leftIntakeServo: CRServo
    // private lateinit var rightIntakeServo: CRServo

    // Encoder ticks per second, not RPM.
    val LAUNCHER_TARGET_VELOCITY = 1250
    val LAUNCHER_MIN_VELOCITY = 1200
    val ARM_UP = 0.95
    val ARM_DOWN = 0.2

    var leftFrontPower = 0.0
    var rightFrontPower = 0.0
    var leftBackPower = 0.0
    var rightBackPower = 0.0
    var intakePower = 0.0

    override fun init() {
        // These names must match the Driver Station hardware configuration.
        leftFrontDrive = hardwareMap.get(DcMotor::class.java, "left_front_drive")
        rightFrontDrive = hardwareMap.get(DcMotor::class.java, "right_front_drive")
        leftBackDrive = hardwareMap.get(DcMotor::class.java, "left_back_drive")
        rightBackDrive = hardwareMap.get(DcMotor::class.java, "right_back_drive")
        intake = hardwareMap.get(DcMotor::class.java, "intake")
        launcher = hardwareMap.get(DcMotorEx::class.java, "launcher")
        windmillServo = hardwareMap.get(CRServo::class.java, "windmillServo")
        arm0 = hardwareMap.get(Servo::class.java, "arm0");
        arm1 = hardwareMap.get(Servo::class.java, "arm1");
        // leftIntakeServo = hardwareMap.get(CRServo::class.java, "left_intake_servo")
        // rightIntakeServo = hardwareMap.get(CRServo::class.java, "right_intake_servo")

        leftFrontDrive.direction = DcMotorSimple.Direction.REVERSE
        rightFrontDrive.direction = DcMotorSimple.Direction.FORWARD
        leftBackDrive.direction = DcMotorSimple.Direction.REVERSE
        rightBackDrive.direction = DcMotorSimple.Direction.FORWARD

        arm1.direction = Servo.Direction.REVERSE

        leftFrontDrive.zeroPowerBehavior = BRAKE
        rightFrontDrive.zeroPowerBehavior = BRAKE
        leftBackDrive.zeroPowerBehavior = BRAKE
        rightBackDrive.zeroPowerBehavior = BRAKE
        intake.zeroPowerBehavior = BRAKE

        // Closed-loop launcher velocity control using its motor encoder.
        launcher.mode = DcMotor.RunMode.RUN_USING_ENCODER
        launcher.setPIDFCoefficients(
            DcMotor.RunMode.RUN_USING_ENCODER,
            PIDFCoefficients(40.0, 0.0, 0.0, 12.5)
        )

        windmillServo.power = 0.0
        windmillServo.direction = DcMotorSimple.Direction.FORWARD
        // leftIntakeServo.power = 0.0
        // rightIntakeServo.power = 0.0
        // rightIntakeServo.direction = DcMotorSimple.Direction.FORWARD
        // leftIntakeServo.direction = DcMotorSimple.Direction.REVERSE

        telemetry.addData("Status", "Initialized")
    }

    override fun start() {
        // Preserve the previous physical starting position after reversing.
        arm0.position = ARM_UP
        arm1.position = ARM_UP
    }

    override fun loop() {
        mecanumDrive(
            -gamepad1.left_stick_y.toDouble(),
            gamepad1.left_stick_x.toDouble(),
            gamepad1.right_stick_x.toDouble()
        )

        intakePower = (gamepad1.right_trigger - gamepad1.left_trigger).toDouble()
        // Launch may add intake power; apply it only after that adjustment.
        launch()
        intake.power = intakePower
        // leftIntakeServo.power = intakePower
        // rightIntakeServo.power = intakePower

        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower)
        telemetry.addData(
            "Triggers", "left (%.2f), right (%.2f)",
            gamepad1.left_trigger, gamepad1.right_trigger
        )

        if (super.gamepad1.yWasPressed()) {
            arm1.position = if (arm1.position <= ARM_DOWN + 0.2) ARM_UP else ARM_DOWN
            arm0.position = arm1.position
        }
    }

    fun mecanumDrive(forward: Double, strafe: Double, rotate: Double) {
        leftFrontPower = forward + strafe + rotate
        rightFrontPower = forward - strafe - rotate
        leftBackPower = forward - strafe + rotate
        rightBackPower = forward + strafe - rotate

        val maxPower = max(
            max(abs(leftFrontPower), abs(rightFrontPower)),
            max(abs(leftBackPower), abs(rightBackPower))
        )
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower
            rightFrontPower /= maxPower
            leftBackPower /= maxPower
            rightBackPower /= maxPower
        }

        leftFrontDrive.power = leftFrontPower
        rightFrontDrive.power = rightFrontPower
        leftBackDrive.power = leftBackPower
        rightBackDrive.power = rightBackPower
    }

    fun launch() {
        launcher.velocity = if (gamepad1.right_bumper)
            LAUNCHER_TARGET_VELOCITY.toDouble() else 0.0

        if (gamepad1.right_bumper && launcher.velocity > LAUNCHER_MIN_VELOCITY) {
            windmillServo.power = 1.0
            intakePower += 0.5
        } else {
            windmillServo.power = 0.0
        }
    }
}
