package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="AlexRajEthanButMore", group="Linear OpMode")
public class AlexRajEthanButMore extends LinearOpMode {

    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    private DcMotorEx shooter;
    private DcMotorEx intake;

    private Servo ramp;

    private boolean shooterOn = false;
    private boolean lastRightBumper = false;

    private int intakeDirection = 0;

    private boolean lastLeftBumper = false;
    private boolean lastB = false;

    private double shooterVelocity = 1200.0;

    private static final double INTAKE_VELOCITY = 800.0;
    private static final double VELOCITY_STEP = 10.0;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    private static final double RAMP_DOWN = 0.35;
    private static final double RAMP_UP = 0.55;

    @Override
    public void runOpMode() {

        frontLeftDrive =
                hardwareMap.get(DcMotor.class, "front_left_drive");

        backLeftDrive =
                hardwareMap.get(DcMotor.class, "back_left_drive");

        frontRightDrive =
                hardwareMap.get(DcMotor.class, "front_right_drive");

        backRightDrive =
                hardwareMap.get(DcMotor.class, "back_right_drive");

        shooter =
                hardwareMap.get(DcMotorEx.class, "shooter");

        intake =
                hardwareMap.get(DcMotorEx.class, "intake");

        ramp =
                hardwareMap.get(Servo.class, "ramp");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        shooter.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setVelocity(0);
        intake.setVelocity(0);

        ramp.setPosition(RAMP_DOWN);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double frontLeftPower =
                    axial + lateral + yaw;

            double frontRightPower =
                    axial - lateral - yaw;

            double backLeftPower =
                    axial - lateral + yaw;

            double backRightPower =
                    axial + lateral - yaw;

            double max = Math.max(
                    Math.max(
                            Math.abs(frontLeftPower),
                            Math.abs(frontRightPower)
                    ),
                    Math.max(
                            Math.abs(backLeftPower),
                            Math.abs(backRightPower)
                    )
            );

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }   

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            if (gamepad2.dpad_up && !lastDpadUp) {
                shooterVelocity += VELOCITY_STEP;
            }

            if (gamepad2.dpad_down && !lastDpadDown) {
                shooterVelocity -= VELOCITY_STEP;
            }

            shooterVelocity = Math.max(0, shooterVelocity);

            lastDpadUp = gamepad2.dpad_up;
            lastDpadDown = gamepad2.dpad_down;

            if (gamepad2.right_bumper && !lastRightBumper) {
                shooterOn = !shooterOn;
            }

            if (shooterOn) {
                shooter.setVelocity(shooterVelocity);
            } else {
                shooter.setVelocity(0);
            }

            lastRightBumper = gamepad2.right_bumper;

            if (gamepad2.left_bumper && !lastLeftBumper) {

                if (intakeDirection == 1) {
                    intakeDirection = 0;
                } else {
                    intakeDirection = 1;
                }
            }

            if (gamepad2.b && !lastB) {

                if (intakeDirection == -1) {
                    intakeDirection = 0;
                } else {
                    intakeDirection = -1;
                }
            }

            intake.setVelocity(
                    INTAKE_VELOCITY * intakeDirection
            );

            lastLeftBumper = gamepad2.left_bumper;
            lastB = gamepad2.b;

            if (gamepad2.left_trigger > 0.1) {
                ramp.setPosition(RAMP_UP);
            } else {
                ramp.setPosition(RAMP_DOWN);
            }

            telemetry.addData(
                    "Shooter Target",
                    "%.0f ticks/sec",
                    shooterVelocity
            );

            telemetry.addData(
                    "Shooter Actual",
                    "%.0f ticks/sec",
                    shooter.getVelocity()
            );

            telemetry.addData(
                    "Shooter",
                    shooterOn ? "ON" : "OFF"
            );

            telemetry.addData(
                    "Intake",
                    intakeDirection == 1
                            ? "FORWARD"
                            : intakeDirection == -1
                            ? "REVERSE"
                            : "OFF"
            );

            telemetry.addData(
                    "Ramp",
                    gamepad2.left_trigger > 0.1
                            ? "UP"
                            : "DOWN"
            );

            telemetry.addData(
                    "Shooter Speed",
                    "D-pad Up/Down = +/- 10"
            );

            telemetry.update();
        }

        shooter.setVelocity(0);
        intake.setVelocity(0);

        ramp.setPosition(RAMP_DOWN);

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}