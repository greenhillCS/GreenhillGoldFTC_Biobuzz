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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Position_Storage;

@TeleOp(name = "AlexRajEthanButMore", group = "Linear OpMode")
public class AlexRajEthanButMore extends LinearOpMode {

    // =====================================================
    // PEDRO PATHING
    // =====================================================

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // (0, 0, 0) = the shot 1 starting spot, the same as PathARed.
    // After PathARed, TeleOp starts from where the auto ended instead.
    private final Pose startPose =
            poseFactory.of(0, 0, 0);

    // Waypoint 1 - Gamepad 1 D-pad Up
    // CHANGED: 360 -> 0. Same direction, but avoids any chance of a full extra spin.
    private final Pose waypoint1 =
            poseFactory.of(0, 0, 0);

    // Waypoint 2 - Gamepad 1 D-pad Down: the shot 2 spot
    // (re-recorded after the Pinpoint offset fix, same as PathARed)
    private final Pose waypoint2 =
            poseFactory.of(-114.65, -2.8, 184.3);

    // Waypoint safety settings
    private static final double WAYPOINT_SPEED = 0.5;       // fraction of max path speed
    private static final double WAYPOINT_TIMEOUT_S = 8.0;   // give up and stop after this
    private static final double STICK_OVERRIDE = 0.3;       // any stick past this cancels auto

    private boolean lastDpadUp1 = false;
    private boolean lastDpadDown1 = false;

    private boolean waypointActive = false;

    // The robot does nothing until the driver touches a drive stick once.
    // Stops it from holding/turning on its own right after the auto.
    private boolean driverHasControl = false;
    private final ElapsedTime waypointTimer = new ElapsedTime();

    private Pose activeWaypoint = null;
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
    private boolean lastDpadLeft2 = false;


    // =====================================================
    // RAMP
    // =====================================================

    private Servo ramp;

    private static final double RAMP_DOWN = 0.22;
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

        // Continue from wherever the robot last was (end of PathARed, or the
        // end of the last TeleOp run). If you moved the robot by hand back to
        // the shot 1 spot, press D-pad Left during INIT to start at (0, 0, 0).
        // After the robot app restarts, it always starts at (0, 0, 0).
        boolean useAutoPose = Position_Storage.fromAuto;

        follower.setPose(useAutoPose ? Position_Storage.pos : startPose);
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

        // Shooter PIDF
        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(100, 0, 0.01, 12)
        );

        shooter.setVelocity(0);
        intake.setVelocity(0);

        ramp.setPosition(RAMP_DOWN);

        // INIT: show which start position is being used.
        // If the robot was picked up and put back at the shot 1 spot after
        // the auto, press D-pad Left on gamepad 1 to start from (0, 0, 0).
        while (!isStarted() && !isStopRequested()) {

            if (useAutoPose && gamepad1.dpad_left) {
                useAutoPose = false;
                follower.setPose(startPose);
                follower.update();
            }

            telemetry.addData("Status", "Initialized");

            if (useAutoPose) {
                Pose p = Position_Storage.pos;
                telemetry.addData("Start Position", "CONTINUING from last run (%.1f, %.1f, %.0f°)",
                        p.x(), p.y(), Math.toDegrees(p.heading()));
                telemetry.addData("Moved the robot by hand to shot 1?", "Press D-pad Left (gamepad 1)");
            } else {
                telemetry.addData("Start Position", "(0, 0, 0) = shot 1 spot");
                telemetry.addData("Check", "Robot must be sitting at the shot 1 spot");
            }

            telemetry.update();
        }

        if (isStopRequested()) {
            return;
        }


        // =====================================================
        // MAIN LOOP
        // =====================================================

        while (opModeIsActive()) {

            // =================================================
            // WAYPOINT CONTROLS - GAMEPAD 1
            // =================================================

            // D-pad Up = Waypoint 1
            if (gamepad1.dpad_up && !lastDpadUp1) {
                goToWaypoint(waypoint1, "Waypoint 1");
            }

            // D-pad Down = Waypoint 2
            if (gamepad1.dpad_down && !lastDpadDown1) {
                goToWaypoint(waypoint2, "Waypoint 2");
            }

            lastDpadUp1 = gamepad1.dpad_up;
            lastDpadDown1 = gamepad1.dpad_down;

            // D-pad Left = Cancel automatic navigation
            if (gamepad1.dpad_left && waypointActive) {
                cancelWaypoint("Cancelled");
            }

            // Touching any drive stick also cancels automatic navigation,
            // so the driver can always take over instantly.
            boolean sticksMoved =
                    Math.abs(gamepad1.left_stick_x) > STICK_OVERRIDE
                            || Math.abs(gamepad1.left_stick_y) > STICK_OVERRIDE
                            || Math.abs(gamepad1.right_stick_x) > STICK_OVERRIDE;

            if (waypointActive && sticksMoved) {
                cancelWaypoint("Driver override");
            }


            // =================================================
            // DRIVE CONTROL
            // =================================================

            if (waypointActive) {

                // Pedro controls the drivetrain
                follower.update();

                // isBusy() stays true until the robot has stopped at the target.
                // After that the follower keeps holding the target pose until
                // the driver touches a stick, which hands control back.
                if (!follower.isBusy()) {
                    waypointActive = false;
                    activeWaypoint = null;
                    targetName = "Arrived";
                } else if (waypointTimer.seconds() > WAYPOINT_TIMEOUT_S) {
                    cancelWaypoint("Timed out");
                }

            } else {

                if (Math.abs(gamepad1.left_stick_x) > 0.1
                        || Math.abs(gamepad1.left_stick_y) > 0.1
                        || Math.abs(gamepad1.right_stick_x) > 0.1) {
                    driverHasControl = true;
                }

                if (driverHasControl) {
                    // Manual joystick control (unchanged)
                    ManualDrive.driveOrHold(
                            follower,
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    );
                } else {
                    // Sit still until the driver takes over
                    follower.manual(0, 0, 0);
                }

                follower.update();
            }


            // =================================================
            // SAVE CURRENT POSITION
            // =================================================

            Pose robotPose = follower.pose();

            Position_Storage.pos = new Pose(
                    robotPose.x(),
                    robotPose.y(),
                    robotPose.heading()
            );
            Position_Storage.fromAuto = true;   // next TeleOp continues from here


            // =================================================
            // SHOOTER SPEED ADJUSTMENT - GAMEPAD 2
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
            // SHOOTER TOGGLE - GAMEPAD 2
            // =================================================

            if (gamepad2.right_bumper && !lastRightBumper) {
                shooterOn = !shooterOn;
            }

            shooter.setVelocity(
                    shooterOn ? shooterVelocity : 0
            );

            lastRightBumper = gamepad2.right_bumper;


            // =================================================
            // INTAKE FORWARD / REVERSE - GAMEPAD 2
            // =================================================

            // Left bumper = toggle forward intake
            if (gamepad2.left_bumper && !lastLeftBumper) {

                intakeDirection =
                        (intakeDirection == 1) ? 0 : 1;
            }

            // D-pad Left = toggle reverse intake
            // CHANGED from B: Start+B assigns User 2, which was toggling the intake.
            if (gamepad2.dpad_left && !lastDpadLeft2) {

                intakeDirection =
                        (intakeDirection == -1) ? 0 : -1;
            }

            intake.setVelocity(
                    INTAKE_VELOCITY * intakeDirection
            );

            lastLeftBumper = gamepad2.left_bumper;
            lastDpadLeft2 = gamepad2.dpad_left;


            // =================================================
            // RAMP CONTROL - GAMEPAD 2
            // =================================================

            if (gamepad2.left_trigger > 0.1) {
                ramp.setPosition(RAMP_UP);
            } else {
                ramp.setPosition(RAMP_DOWN);
            }


            // =================================================
            // TELEMETRY
            // =================================================

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

            // Used for the spin test: turn in place and watch this number.
            telemetry.addData(
                    "Distance From Start",
                    "%.2f inches",
                    Math.hypot(robotPose.x(), robotPose.y())
            );

            telemetry.addData(
                    "Navigation",
                    waypointActive ? "AUTO (sticks or D-pad Left cancel)" : "Manual"
            );

            if (waypointActive && activeWaypoint != null) {

                double distance = Math.hypot(
                        activeWaypoint.x() - robotPose.x(),
                        activeWaypoint.y() - robotPose.y()
                );

                telemetry.addData(
                        "Distance To Waypoint",
                        "%.2f inches",
                        distance
                );

                telemetry.addData(
                        "Target X",
                        "%.2f",
                        activeWaypoint.x()
                );

                telemetry.addData(
                        "Target Y",
                        "%.2f",
                        activeWaypoint.y()
                );

                telemetry.addData(
                        "Target Heading",
                        "%.1f degrees",
                        Math.toDegrees(activeWaypoint.heading())
                );

                telemetry.addData(
                        "Auto Time",
                        "%.1f s",
                        waypointTimer.seconds()
                );
            }


            // =================================================
            // SHOOTER TELEMETRY
            // =================================================

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


            // =================================================
            // INTAKE TELEMETRY
            // =================================================

            telemetry.addData(
                    "Intake",
                    intakeDirection == 1
                            ? "FORWARD"
                            : intakeDirection == -1
                            ? "REVERSE"
                            : "OFF"
            );


            // =================================================
            // RAMP TELEMETRY
            // =================================================

            telemetry.addData(
                    "Ramp",
                    gamepad2.left_trigger > 0.1
                            ? "UP"
                            : "DOWN"
            );


            // =================================================
            // WAYPOINT TELEMETRY
            // =================================================

            telemetry.addData(
                    "Waypoint 1",
                    "D-pad Up: (%.2f, %.2f, %.1f°)",
                    waypoint1.x(),
                    waypoint1.y(),
                    Math.toDegrees(waypoint1.heading())
            );

            telemetry.addData(
                    "Waypoint 2",
                    "D-pad Down: (%.2f, %.2f, %.1f°)",
                    waypoint2.x(),
                    waypoint2.y(),
                    Math.toDegrees(waypoint2.heading())
            );

            telemetry.addData(
                    "Cancel",
                    "D-pad Left or move any stick"
            );

            telemetry.update();
        }


        // =====================================================
        // STOP
        // =====================================================

        follower.stop();

        shooter.setVelocity(0);
        intake.setVelocity(0);

        ramp.setPosition(RAMP_DOWN);
    }


    // =====================================================
    // AUTOMATIC WAYPOINT NAVIGATION
    // =====================================================

    private void goToWaypoint(Pose target, String name) {

        driverHasControl = true;

        Pose currentPose = follower.pose();

        activeWaypoint = target;
        targetName = name;

        waypointActive = true;
        waypointTimer.reset();

        // If already at the waypoint, just correct position and heading
        double distance = Math.hypot(
                target.x() - currentPose.x(),
                target.y() - currentPose.y()
        );

        if (distance < 1.0) {
            follower.hold(target);
            return;
        }

        // CHANGED from .constant(currentPose), which ignored the waypoint's
        // heading, so the robot never turned to it.
        // Now the robot turns to the waypoint heading while it drives,
        // always taking the shorter way around (never more than 180 degrees).
        double startHeading = currentPose.heading();
        double turn = target.heading() - startHeading;
        turn = Math.atan2(Math.sin(turn), Math.cos(turn));
        double endHeading = startHeading + turn;

        Path path = Paths
                .line(currentPose, target)
                .linear(startHeading, endHeading);

        // Speed cap while testing. Raise WAYPOINT_SPEED once it behaves.
        // If this line does not compile on your Pedro version,
        // replace it with: follower.follow(path);
        follower.follow(path.with(Constants.foresightConfig.maxPathSpeed.at(WAYPOINT_SPEED)));
    }

    private void cancelWaypoint(String reason) {

        follower.stop();

        waypointActive = false;
        activeWaypoint = null;
        targetName = reason;
    }
}
