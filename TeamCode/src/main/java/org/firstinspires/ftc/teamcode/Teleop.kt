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
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import kotlin.math.max

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
@TeleOp(name = "Mec BioBuzz StarterBot Teleop", group = "StarterBot") //@Disabled
class BioBuzzStarterbotTeleopMecanum : OpMode() {
    // Declare OpMode members.
    private lateinit var leftFrontDrive: DcMotor
    private lateinit var leftBackDrive: DcMotor
    private lateinit var rightFrontDrive: DcMotor
    private lateinit var rightBackDrive: DcMotor
    private lateinit var launcher: DcMotorEx
    private lateinit var intake: DcMotor
    private lateinit var shoot_stop: DcMotor


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
    val LAUNCHER_TARGET_VELOCITY: Int = 4500
    val LAUNCHER_MIN_VELOCITY: Int = 4450


    /*
     * These four variables store the power we need to apply to the motors. In other cases, we may
     * choose to declare these variables inside the mecanumDrive() function, instead we declare them
     * here so that we can access them in our main loop for telemetry.
     */
    var leftFrontPower: Double = 0.0
    var rightFrontPower: Double = 0.0
    var leftBackPower: Double = 0.0
    var rightBackPower: Double = 0.0
    var flip: Boolean = true;

    // Create a variable to set to the intake.
    var intakePower: Double = 0.0

    /*
     * Code to run ONCE when the driver hits INIT
     */
    override fun init() {
        /*
                 * Initialize the hardware variables. Note that the strings used here as parameters
                 * to 'get' must correspond to the names assigned during the robot configuration
                 * step.
                 */

        leftFrontDrive = hardwareMap.get(DcMotor::class.java, "left_front")
        rightFrontDrive = hardwareMap.get(DcMotor::class.java, "right_front")
        leftBackDrive = hardwareMap.get(DcMotor::class.java, "left_back")
        rightBackDrive = hardwareMap.get(DcMotor::class.java, "right_back")
        intake = hardwareMap.get(DcMotor::class.java, "intake")
        launcher = hardwareMap.get(DcMotorEx::class.java, "launcher")
        shoot_stop = hardwareMap.get(DcMotor::class.java, "shoot_stop")


        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.direction = DcMotorSimple.Direction.REVERSE
        rightFrontDrive.direction = DcMotorSimple.Direction.FORWARD
        leftBackDrive.direction = DcMotorSimple.Direction.REVERSE
        rightBackDrive.direction = DcMotorSimple.Direction.FORWARD
        shoot_stop.direction = DcMotorSimple.Direction.REVERSE
        launcher.direction = DcMotorSimple.Direction.REVERSE

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
        rightFrontDrive.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
        leftBackDrive.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
        rightBackDrive.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
        intake.zeroPowerBehavior = ZeroPowerBehavior.BRAKE

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher!!.mode = DcMotor.RunMode.RUN_USING_ENCODER

        launcher!!.setPIDFCoefficients(
            DcMotor.RunMode.RUN_USING_ENCODER,
            PIDFCoefficients(40.0, 0.0, 0.0, 12.5)
        )

        /*
         * set Feeders to an initial value to initialize the servo controller
         */

        /*
         * Much like our drivetrain motors, we set the right intake servo to reverse so that both
         * servos work to pull elements into the intake.
         */

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized")
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    override fun init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    override fun start() {
        shoot_stop.power = 1.0;
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    override fun loop() {
        /*
         * Here we call a function called mecanumDrive. The mecanumDrive function takes the input from
         * the joysticks, and applies power to the drive motors to move the robot as requested
         * by the driver. Moving the left joystick forwards/back moves all motors forwards/back,
         * moving the right joystick left/right rotates the robot clockwise or counterclockwise,
         * and moving the left joystick left moves the motors in the right way to create a sideways
         * "strafe" movement. Combinations of these inputs can be used to create more complex maneuvers.
         * Note, moving the joystick forward on most gamepads results in a negative signal, so
         * we invert it before passing it to the function.
         */
        mecanumDrive(
            gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble(),
            -gamepad1.right_stick_x.toDouble()
        )

        /*
         * Set the intake power variable to equal the right trigger, minus the left trigger.
         * Each trigger outputs a signal from 0-1, with 0 as fully released, and 1 fully depressed.
         * This gives us proportional control of the intake speed. The speed increases as we pull
         * the right trigger further. It's occasionally helpful to be able to reverse the intake,
         * so we also factor in the left trigger. If the left trigger is fully depressed,
         * the intakePower variable will be -1. If the right trigger is fully depressed, the variable
         * will be 1. If the driver pulls both triggers, the intake will remain off.
         * We use this technique (creating a variable, and setting it to our control inputs) to
         * allow us to avoid setting the same motors/servos power more than once per loop. That can
         * create erratic behavior.
         */

        launch()

        /*
         * Here we set our intake motor and servos to their intake power. The order of operations
         * here is important though. The gamepad triggers define the starting point for the intake
         * power variable in each loop of our code, but inside our launch function we also sometimes
         * change the intake power. So we need to give our launch function a chance to modify the
         * variable before we write it to our motor and servos.
         */
        if (super.gamepad1.right_bumper) {
            intake.power = 1.0
        } else if (super.gamepad1.left_bumper) {
            intake.power = -1.0
        } else if (!super.gamepad1.right_trigger_pressed) {
            intake.power = 0.0
        }

        /*
         * Show motor powers on the Driver Station via telemetry.
         */
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower)
        telemetry.addData(
            "Triggers",
            "left (%.2f, right (%.2f)",
            gamepad1.left_trigger,
            gamepad1.right_trigger
        )
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    override fun stop() {
    }

    fun mecanumDrive(forward: Double, strafe: Double, rotate: Double) {
        leftFrontPower = forward + strafe - if (flip) -rotate else rotate
        rightFrontPower = forward - strafe + if (flip) -rotate else rotate
        leftBackPower = forward - strafe - if (flip) -rotate else rotate
        rightBackPower = forward + strafe + if (flip) -rotate else rotate

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

    fun launch() {
        /*
         * Calling gamepad1.right_bumper returns a boolean which will be true if the bumper is
         * held down, and false if it is not. Notably, this will continue to be true for every
         * cycle of our code that the driver holds down that bumper.
         * The first step of our launch() function is checking to see if the user is currently
         * holding down the right gamepad. If they are, then we want to start spinning up the launcher.
         * Otherwise, we start spinning the launcher down.
         */
        if (gamepad1.right_trigger_pressed) {
            launcher.velocity = LAUNCHER_TARGET_VELOCITY.toDouble()
            if (launcher.velocity > LAUNCHER_MIN_VELOCITY) {
                shoot_stop.power = -0.35
                intake.power = 1.0
            }
        } else {
            launcher.velocity = 0.0
            shoot_stop.power = 1.0
        }
    }
}