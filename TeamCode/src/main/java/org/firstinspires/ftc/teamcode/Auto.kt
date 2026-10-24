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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import kotlin.math.max
import kotlin.time.Duration.Companion.seconds

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot with Mecanum
 * Wheels for the 2026-2027 FIRST® Tech Challenge. On top of a mecanum wheel drivetrain, it uses
 * one motor driving an intake roller, two servos which pull elements out of corners, and a high-speed
 * launcher motor.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */
@Autonomous(name = "Auto", group = "StarterBot") //@Disabled
class Auto : LinearOpMode() {
    // Declare OpMode members.
    private lateinit var leftFrontDrive: DcMotor
    private lateinit var leftBackDrive: DcMotor
    private lateinit var rightFrontDrive: DcMotor
    private lateinit var rightBackDrive: DcMotor
    private lateinit var launcher: DcMotorEx
    private lateinit var intake: DcMotor
    private lateinit var leftIntakeServo: CRServo
    private lateinit var rightIntakeServo: CRServo
    private lateinit var windmillServo: CRServo


    /*
     * These two variables are used to control the velocity of the launcher motor.
     * They are both in encoder ticks per second. The motors we use in the FIRST Tech Challenge
     * have encoders with a resolution of 28 ticks per revolution. We can convert this to RPM
     * by dividing the value by 28, to get to revolutions per second, before multiplying by 60
     * to get revolutions per minute.
     * We pass the target velocity variable to our motor to set the goal. We use the min velocity
     * in the launch() function to only run the windmill servo when the motor is spinning fast
     * enough to make a successful throw.
     */
    val LAUNCHER_TARGET_VELOCITY: Int = 1320 //2678 RPM
    val LAUNCHER_MIN_VELOCITY: Int = 1290 //2571 RPM


    /*
     * These four variables store the power we need to apply to the motors. In other cases, we may
     * choose to declare these variables inside the mecanumDrive() function, instead we declare them
     * here so that we can access them in our main loop for telemetry.
     */
    var leftFrontPower: Double = 0.0
    var rightFrontPower: Double = 0.0
    var leftBackPower: Double = 0.0
    var rightBackPower: Double = 0.0
    var flip: Boolean = false;

    // Create a variable to set to the intake.
    var intakePower: Double = 0.0

    /*
     * Code to run ONCE when the driver hits INIT
     */
    override fun runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor::class.java, "left_front_drive")
        rightFrontDrive = hardwareMap.get(DcMotor::class.java, "right_front_drive")
        leftBackDrive = hardwareMap.get(DcMotor::class.java, "left_back_drive")
        rightBackDrive = hardwareMap.get(DcMotor::class.java, "right_back_drive")
        intake = hardwareMap.get(DcMotor::class.java, "intake")
        launcher = hardwareMap.get(DcMotorEx::class.java, "launcher")
        windmillServo = hardwareMap.get(CRServo::class.java, "windmillServo")
        leftIntakeServo = hardwareMap.get(CRServo::class.java, "left_intake_servo")
        rightIntakeServo = hardwareMap.get(CRServo::class.java, "right_intake_servo")

        leftFrontDrive.direction = DcMotorSimple.Direction.REVERSE
        rightFrontDrive.direction = DcMotorSimple.Direction.FORWARD
        leftBackDrive.direction = DcMotorSimple.Direction.REVERSE
        rightBackDrive.direction = DcMotorSimple.Direction.FORWARD


        rightIntakeServo.direction = DcMotorSimple.Direction.REVERSE
        windmillServo.direction = DcMotorSimple.Direction.FORWARD

        leftFrontDrive.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
        rightFrontDrive.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
        leftBackDrive.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
        rightBackDrive.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
        intake.zeroPowerBehavior = ZeroPowerBehavior.BRAKE

        launcher.mode = DcMotor.RunMode.RUN_USING_ENCODER

        launcher.setPIDFCoefficients(
            DcMotor.RunMode.RUN_USING_ENCODER,
            PIDFCoefficients(40.0, 0.0, 0.0, 12.5)
        )

        leftIntakeServo.power = 0.0
        rightIntakeServo.power = 0.0
        windmillServo.power = 0.0

        super.telemetry.addData("Status", "Initialized")
        super.telemetry.update()

        var team_blue = false;
        var no_shoot = false;
        var park_only = false;
        while (super.opModeInInit()) {
            super.telemetry.clearAll();
            super.telemetry.addData("Status", "Initialized")
            super.telemetry.addData("Team", if (team_blue) "BLUE" else "RED")
            super.telemetry.addData("No shoot", no_shoot)
            super.telemetry.addData("Park only", park_only)
            super.telemetry.addLine("Press X for blue team and B for red team");
            super.telemetry.addLine("Press Y to toggle no shoot auto");
            super.telemetry.addLine("Press A to toggle Park only mode");
            super.telemetry.update();

            if (super.gamepad1.x) team_blue = true;
            else if (super.gamepad1.b) team_blue = false;

            if (super.gamepad1.yWasPressed()) no_shoot = !no_shoot;
            if (super.gamepad1.aWasPressed()) park_only = !park_only
        }

        super.waitForStart();

        super.telemetry.clearAll();

        super.telemetry.addData("Status", "Running");
        super.telemetry.addData("State", "S1 (Line up)");

        super.telemetry.update()

        if (!(no_shoot || park_only)) {
            val shootHold = 0.85
            val shootDelay = 0.3
            val amount = 4;

            var shootTime = ElapsedTime()
            for (i in 0..<amount) {
                super.telemetry.addData("balls shot", i)
                super.telemetry.update()
                while (shootTime.seconds() <= shootHold) {
                    launch(true)
                    if (super.isStopRequested) return
                }
                shootTime = ElapsedTime()
                while (shootTime.seconds() <= shootDelay) {
                    windmillServo.power = 0.0
                    if (super.isStopRequested) return
                }
            }
            launch(false)

            shootTime = ElapsedTime()
            while (shootTime.seconds() <= shootDelay) {
                if (super.isStopRequested) return
            }
        }

        var driveTime = ElapsedTime()

        if (park_only) {
            val forwardSeconds = 0.2;
            val strafeSeconds = 0.5;

            mecanumDrive(-0.75, 0.0, 0.0);
            while (driveTime.seconds() <= forwardSeconds) {
                if (super.isStopRequested) return
            }


            driveTime = ElapsedTime()
            mecanumDrive(0.0, 0.75, 0.0);
            while (driveTime.seconds() <= strafeSeconds) {
                if (super.isStopRequested) return
            }

            driveTime = ElapsedTime()
            mecanumDrive(0.75, 0.0, 0.0);
            while (driveTime.seconds() <= forwardSeconds) {
                if (super.isStopRequested) return
            }
        } else {
            val forwardSeconds = 2.4
            val strafeSeconds = 0.45
            val toWallSeconds = 0.50

            driveTime = ElapsedTime()
            mecanumDrive(0.0, 0.75, 0.0)
            while (driveTime.seconds() <= toWallSeconds) {
                if (super.isStopRequested) return
            }

            mecanumDrive(-0.75, 0.0, 0.0);
            while (driveTime.seconds() <= forwardSeconds) {
                if (super.isStopRequested) return
            }


            driveTime = ElapsedTime()
            mecanumDrive(0.0, 0.75, 0.0);
            while (driveTime.seconds() <= strafeSeconds) {
                if (super.isStopRequested) return
            }
        }


    }

    fun mecanumDrive(forward: Double, strafe: Double, rotate: Double) {
        leftFrontPower = forward + strafe + rotate
        rightFrontPower = forward - strafe - rotate
        leftBackPower = forward - strafe + rotate
        rightBackPower = forward + strafe - rotate

        var max = max(abs(leftFrontPower), abs(rightFrontPower))
        max = max(max, abs(leftBackPower))
        max = max(max, abs(rightBackPower))

        if (max > 1.0) {
            leftFrontPower /= max
            rightFrontPower /= max
            leftBackPower /= max
            rightBackPower /= max
        }

        if (flip) {
            leftFrontPower *= -1;
            rightFrontPower *= -1;
            leftBackPower *= -1;
            rightBackPower *= -1;
        }

        /*
         * Send calculated power to wheels
         */
        leftFrontDrive!!.power = leftFrontPower
        rightFrontDrive!!.power = rightFrontPower
        leftBackDrive!!.power = leftBackPower
        rightBackDrive!!.power = rightBackPower
    }

    fun launch(should: Boolean) {
        /*
         * Calling gamepad1.right_bumper returns a boolean which will be true if the bumper is
         * held down, and false if it is not. Notably, this will continue to be true for every
         * cycle of our code that the driver holds down that bumper.
         * The first step of our launch() function is checking to see if the user is currently
         * holding down the right gamepad. If they are, then we want to start spinning up the launcher.
         * Otherwise, we start spinning the launcher down.
         */
        if (should) {
            launcher!!.velocity = LAUNCHER_TARGET_VELOCITY.toDouble()
        }

        /*
         * Here we ask if the driver is currently pressing the right bumper, AND the launcher is
         * spinning fast enough to make a successful shot. If it is, then we will turn on the
         * windmill servo to start feeding the elements into the launcher motor. We also
         * add some power to the intake power. This can sometimes help dislodge stuck elements from
         * inside the hopper.
         */
        if (should && launcher!!.velocity > LAUNCHER_MIN_VELOCITY) {
            windmillServo!!.power = 1.0
        } else {
            windmillServo!!.power = 0.0
        }
    }
}