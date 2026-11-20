
package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Position_Storage;


@TeleOp(name = "AlexRajEthanButMore", group = "Linear OpMode")
public class AlexRajEthanButMore extends LinearOpMode {

    // =====================================================
    // PEDRO PATHING
    // =====================================================

    private Follower follower;

    private final PoseFactory poseFactory =
            PoseFactory.degrees();

    // Robot's position when TeleOp initializes
    private final Pose startPose =
            poseFactory.of(0, 0, 0);

    // Gamepad 1 A
    private final Pose waypoint1 =
            poseFactory.of(0, 0, 0);

    // Gamepad 1 X
    private final Pose waypoint2 =
            poseFactory.of(105.79, 4.95, 185.9);

    private boolean lastWaypointA = false;
    private boolean lastWaypointX = false;

    private String targetName = "None";


    // =====================================================
    // SHOOTER / INTAKE
    // =====================================================

    private DcMotorEx shooter;
    private DcMotorEx intake;

    private boolean shooterOn = false;
    private boolean lastRightBumper = false;

    // 1 = forward, 0 = off, -1 = reverse
    private int intakeDirection = 0;

    private boolean lastLeftBumper = false;
    private boolean lastB = false;


    // =====================================================
    // RAMP
    // =====================================================

    private Servo ramp;

    private static final double RAMP_DOWN = 0.25;
    private static final double RAMP_UP = 0.45;


    // =====================================================
    // VELOCITIES
    // =====================================================

    private double shooterVelocity = 1200.0;

    private static final double INTAKE_VELOCITY = 800.0;
    private static final double VELOCITY_STEP = 10.0;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;


    // =====================================================
    // RUN OPMODE
    // =====================================================

    @Override
    public void runOpMode() {

        // =================================================
        // PEDRO INITIALIZATION
        // =================================================

        follower = Constants.create(hardwareMap);

        follower.setPose(startPose);
        follower.update();


        // =================================================
        // HARDWARE INITIALIZATION
        // =================================================

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        ramp = hardwareMap.get(Servo.class, "ramp");

        shooter.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setVelocity(0);
        intake.setVelocity(0);
        ramp.setPosition(RAMP_DOWN);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Starting Pose", "(0, 0, 0)");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }


        // =================================================
        // MAIN LOOP
        // =================================================

        while (opModeIsActive()) {

            // =================================================
            // WAYPOINT CONTROLS
            // =================================================

            // Gamepad 1 A = waypoint 1
            if (gamepad1.a && !lastWaypointA) {
                goToWaypoint(waypoint1, "Waypoint 1");
            }

            // Gamepad 1 X = waypoint 2
            if (gamepad1.x && !lastWaypointX) {
                goToWaypoint(waypoint2, "Waypoint 2");
            }

            lastWaypointA = gamepad1.a;
            lastWaypointX = gamepad1.x;

            // Gamepad 1 B = cancel path
            if (gamepad1.b) {
                follower.stop();
                targetName = "Cancelled";
            }


            // =================================================
            // DRIVE CONTROL
            // =================================================

            switch (follower.mode()) {

                case FOLLOW:

                    // Pedro controls drivetrain automatically
                    follower.update();
                    break;

                case MANUAL:
                case IDLE:
                case HOLD:

                    // Manual joystick control
                    ManualDrive.driveOrHold(
                            follower,
                            gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    );

                    follower.update();
                    break;
            }


            // =================================================
            // SAVE CURRENT POSITION
            // =================================================

            Position_Storage.pos = new Pose(
                    follower.pose().x(),
                    follower.pose().y(),
                    follower.pose().heading()
            );


            // =================================================
            // SHOOTER SPEED ADJUSTMENT
            // =================================================

            if (gamepad2.dpad_up && !lastDpadUp) {
                shooterVelocity += VELOCITY_STEP;
            }

            if (gamepad2.dpad_down && !lastDpadDown) {
                shooterVelocity -= VELOCITY_STEP;
            }

            shooterVelocity = Math.max(0, shooterVelocity);

            lastDpadUp = gamepad2.dpad_up;
            lastDpadDown = gamepad2.dpad_down;


            // =================================================
            // SHOOTER TOGGLE
            // =================================================

            if (gamepad2.right_bumper && !lastRightBumper) {
                shooterOn = !shooterOn;
            }

            shooter.setVelocity(
                    shooterOn ? shooterVelocity : 0
            );

            lastRightBumper = gamepad2.right_bumper;


            // =================================================
            // INTAKE FORWARD / REVERSE
            // =================================================

            // Left bumper = forward toggle
            if (gamepad2.left_bumper && !lastLeftBumper) {
                intakeDirection =
                        (intakeDirection == 1) ? 0 : 1;
            }

            // B = reverse toggle
            if (gamepad2.b && !lastB) {
                intakeDirection =
                        (intakeDirection == -1) ? 0 : -1;
            }

            intake.setVelocity(
                    INTAKE_VELOCITY * intakeDirection
            );

            lastLeftBumper = gamepad2.left_bumper;
            lastB = gamepad2.b;


            // =================================================
            // RAMP CONTROL
            // =================================================

            if (gamepad2.left_trigger > 0.1) {
                ramp.setPosition(RAMP_UP);
            } else {
                ramp.setPosition(RAMP_DOWN);
            }


            // =================================================
            // TELEMETRY
            // =================================================

            Pose robotPose = follower.pose();

            telemetry.addData(
                    "Follower Mode",
                    follower.mode()
            );

            telemetry.addData(
                    "Target",
                    targetName
            );

            telemetry.addData(
                    "Robot X",
                    "%.2f",
                    robotPose.x()
            );

            telemetry.addData(
                    "Robot Y",
                    "%.2f",
                    robotPose.y()
            );

            telemetry.addData(
                    "Robot Heading",
                    "%.1f degrees",
                    Math.toDegrees(robotPose.heading())
            );

            if (follower.currentPath() != null) {
                telemetry.addData(
                        "Distance To Waypoint",
                        "%.2f",
                        follower.distanceToEndpoint()
                );
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
                    "Waypoint 1",
                    "Gamepad 1 A: (0, 0, 0)"
            );

            telemetry.addData(
                    "Waypoint 2",
                    "Gamepad 1 X: (105.79, 4.95, 185.9)"
            );

            telemetry.addData(
                    "Cancel",
                    "Gamepad 1 B"
            );

            telemetry.update();
        }


        // =================================================
        // STOP
        // =================================================

        follower.stop();

        shooter.setVelocity(0);
        intake.setVelocity(0);

        ramp.setPosition(RAMP_DOWN);
    }


    // =====================================================
    // AUTOMATIC WAYPOINT NAVIGATION
    // =====================================================

    private void goToWaypoint(Pose target, String name) {

        // Current robot position from Pinpoint
        Pose currentPose = follower.pose();

        // Create straight path from current location
        // to the selected waypoint
        Path path = Paths
                .line(currentPose, target)
                .linear(currentPose, target);

        // Start following the path
        follower.follow(path);

        targetName = name;
    }
}
