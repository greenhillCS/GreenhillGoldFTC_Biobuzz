package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="AlexRajEthanButMore", group="Linear OpMode")
public class AlexRajEthanButMore extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();


    // ================= DRIVE MOTORS =================

    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;


    // ================= SHOOTER / INTAKE =================

    private DcMotorEx shooter;
    private DcMotorEx intake;


    // ================= SERVO =================

    private Servo gateServo;


    // ================= VELOCITY SETTINGS =================
    // Units = encoder ticks per second

    private double shooterVelocity = 1700.0;
    private double intakeVelocity = 800.0;

    private static final double VELOCITY_STEP = 10.0;


    // ================= BUTTON MEMORY =================
    // Prevents holding a button from changing velocity
    // hundreds of times per secondn

    private boolean lastTriangle = false;
    private boolean lastSquare = false;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;


    // ================= GATE =================

    private static final double GATE_CLOSED = 0.70;
    private static final double GATE_OPEN = 0.4;

    private boolean lastA = false;
    private boolean gateFiring = false;
    private double gateStartTime = 0;

    private static final double GATE_OPEN_TIME = 0.50;


    @Override
    public void runOpMode() {

        // ================= HARDWARE =================

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


        gateServo =
                hardwareMap.get(Servo.class, "gateServo");


        // ================= DIRECTIONS =================

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);


        shooter.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);


        // ================= ENCODER MODE =================

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        shooter.setVelocity(0);
        intake.setVelocity(0);

        gateServo.setPosition(GATE_CLOSED);


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        runtime.reset();


        while (opModeIsActive()) {


            // ==================================================
            // DRIVE
            // GAMEPAD 1
            // ==================================================

            double axial =
                    -gamepad1.left_stick_y;

            double lateral =
                    gamepad1.left_stick_x;

            double yaw =
                    gamepad1.right_stick_x;


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



            // ==================================================
            // SHOOTER VELOCITY ADJUSTMENT
            // GAMEPAD 2
            // ==================================================

            // Triangle = +10 ticks/sec
            if (gamepad2.y && !lastTriangle) {

                shooterVelocity += VELOCITY_STEP;
            }


            // Square = -10 ticks/sec
            if (gamepad2.x && !lastSquare) {

                shooterVelocity -= VELOCITY_STEP;
            }


            // Don't let it go below 0
            shooterVelocity =
                    Math.max(0, shooterVelocity);



            // ==================================================
            // INTAKE VELOCITY ADJUSTMENT
            // ==================================================

            // D-pad Up = +100 ticks/sec
            if (gamepad2.dpad_up && !lastDpadUp) {

                intakeVelocity += VELOCITY_STEP;
            }


            // D-pad Down = -100 ticks/sec
            if (gamepad2.dpad_down && !lastDpadDown) {

                intakeVelocity -= VELOCITY_STEP;
            }


            intakeVelocity =
                    Math.max(0, intakeVelocity);



            // ==================================================
            // SHOOTER
            // Right trigger
            // ==================================================

            if (gamepad2.right_trigger > 0.1) {

                shooter.setVelocity(shooterVelocity);

            } else {

                shooter.setVelocity(0);
            }



            // ==================================================
            // INTAKE
            // Left bumper
            // ==================================================

            if (gamepad2.left_bumper) {

                intake.setVelocity(intakeVelocity);

            } else {

                intake.setVelocity(0);
            }



            // ==================================================
            // GATE SERVO
            // Left trigger
            // ==================================================

            if (gamepad2.a && !lastA && !gateFiring) {
                gateFiring = true;
                gateStartTime = runtime.seconds();
            }

            if (gateFiring) {
                gateServo.setPosition(GATE_OPEN);

                if (runtime.seconds() - gateStartTime >= GATE_OPEN_TIME) {
                    gateServo.setPosition(GATE_CLOSED);
                    gateFiring = false;
                }
            } else {
                gateServo.setPosition(GATE_CLOSED);
            }

            lastA = gamepad2.a;



            // ==================================================
            // REMEMBER BUTTON STATES
            // ==================================================

            lastTriangle = gamepad2.y;
            lastSquare = gamepad2.x;

            lastDpadUp = gamepad2.dpad_up;
            lastDpadDown = gamepad2.dpad_down;



            // ==================================================
            // TELEMETRY
            // ==================================================

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
                    "Intake Target",
                    "%.0f ticks/sec",
                    intakeVelocity
            );

            telemetry.addData(
                    "Intake Actual",
                    "%.0f ticks/sec",
                    intake.getVelocity()
            );


            telemetry.addData(
                    "Shooter",
                    gamepad2.right_trigger > 0.1
                            ? "ON"
                            : "OFF"
            );

            telemetry.addData(
                    "Intake",
                    gamepad2.left_bumper
                            ? "ON"
                            : "OFF"
            );

            telemetry.addData(
                    "Gate",
                    gateFiring ? "OPEN" : "CLOSED"
            );


            telemetry.addData(
                    "Shooter +/-",
                    "Triangle / Square"
            );

            telemetry.addData(
                    "Intake +/-",
                    "D-pad Up / Down"
            );


            telemetry.update();
        }


        // ================= STOP EVERYTHING =================

        shooter.setVelocity(0);
        intake.setVelocity(0);

        gateServo.setPosition(GATE_CLOSED);

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}