package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs
import kotlin.math.max

@TeleOp(name = "Basic Drive")
class MecanumDrive : LinearOpMode() {
    lateinit var leftBack: DcMotor;
    lateinit var rightBack: DcMotor;
    lateinit var leftFront: DcMotor;
    lateinit var rightFront: DcMotor;
    lateinit var intake: DcMotor

    override fun runOpMode() {
        leftBack = super.hardwareMap.dcMotor.get("leftBack");
        rightBack = super.hardwareMap.dcMotor.get("rightBack");
        leftFront = super.hardwareMap.dcMotor.get("leftFront");
        rightFront = super.hardwareMap.dcMotor.get("rightFront");
        intake = super.hardwareMap.dcMotor.get("intake");

        intake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        rightFront.direction = DcMotorSimple.Direction.REVERSE;
        rightBack.direction = DcMotorSimple.Direction.REVERSE;

        leftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        rightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        leftBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        rightBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

        leftFront.mode = DcMotor.RunMode.RUN_USING_ENCODER;
        rightFront.mode = DcMotor.RunMode.RUN_USING_ENCODER;
        leftBack.mode = DcMotor.RunMode.RUN_USING_ENCODER;
        rightBack.mode = DcMotor.RunMode.RUN_USING_ENCODER;

        super.telemetry.addData("Status", "Initialized");
        waitForStart();

        if (isStopRequested) return


        while (opModeIsActive()) {
            mecanumDrive(super.gamepad1.left_stick_y, super.gamepad1.left_stick_x, super.gamepad1.right_stick_x);

            if (super.gamepad1.leftTriggerWasPressed()) {
                if (intake.power != 1.0) {
                    intake.power = 1.0
                } else {
                    intake.power = 0.0
                }
            }
            else if (super.gamepad1.rightTriggerWasPressed()) {
                if (intake.power != -1.0) {
                    intake.power = -1.0
                } else {
                    intake.power = 0.0
                }
            }
        }
    }

    fun mecanumDrive(forward: Float, strafe: Float, rotate: Float) {
        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */

        val denominator = max(abs(forward.toDouble()) + abs(strafe.toDouble()) + abs(rotate.toDouble()), 1.0)

        val frontLeft = (-forward + strafe + rotate) / denominator
        val frontRight = (-forward - strafe - rotate) / denominator
        val backLeft = (-forward - strafe + rotate) / denominator
        val backRight = (-forward + strafe - rotate) / denominator

        leftFront.power = frontLeft
        rightFront.power = frontRight
        leftBack.power = backLeft
        rightBack.power = backRight
    }
}