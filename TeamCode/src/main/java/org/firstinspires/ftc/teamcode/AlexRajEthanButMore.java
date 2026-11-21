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

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Position_Storage;

@TeleOp(name = "AlexRajEthanButMore", group = "Linear OpMode")
public class AlexRajEthanButMore extends LinearOpMode {

    // =====================================================
    // PEDRO PATHING
    // =====================================================

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // =====================================================
    // RECORDED SPOTS (same numbers as PathARed)
    // =====================================================
    // (0, 0, 0) = the shot 1 spot, where the auto starts.
    private final Pose shot1Spot   = poseFactory.of(0, 0, 0);
    private final Pose loadingSpot = poseFactory.of(3.03, -47.37, 5.5);
    private final Pose besideStand = poseFactory.of(-73.31, -30.73, 342);
    private final Pose shot2Spot   = poseFactory.of(-114.87, -2.84, 184.4);
    private final Pose parkPose    = poseFactory.of(-106.79, -51.95, 93);

    // Waypoints drive the SAME lanes the auto drives (around the goal stand
    // on the red side, straight in and out at both shot spots), never a
    // straight line through the middle of the field.
    private Pose[] lane;          // shot 1 side -> shot 2 side
    private Pose shot2Stop;       // 2" short of shot 2, like the auto

    private static final double STRAIGHT_IN_SPEED = 0.3;    // last push into a shot spot
    private static final double STRAIGHT_IN_LIMIT_S = 2.0;  // stop pushing after this

    // Waypoint safety settings
    private static final double WAYPOINT_SPEED = 0.5;       // fraction of max path speed
    private static final double WAYPOINT_TIMEOUT_S = 15.0;  // give up and stop after this
    private static final double STICK_OVERRIDE = 0.3;       // any stick past this cancels auto

    // 0 = none, 1 = driving the lane, 2 = settling, 3 = straight push in
    private int stage = 0;
    private boolean goingToShot2 = false;
    private double settleSeconds = 0.3;
    private final ElapsedTime stageTimer = new ElapsedTime();

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

        buildLane();

        // START POSITION
        // Default: exactly where the auto stopped (if PathARed ran since the
        // robot app started), otherwise the park spot.
        // During INIT: D-pad Left = robot is on the shot 1 spot instead,
        //              D-pad Right = back to the default.
        Pose autoEnd = null;
        if (Position_Storage.fromAuto && Position_Storage.pos != null) {
            Pose saved = Position_Storage.pos;
            autoEnd = new Pose(saved.x(), saved.y(), saved.heading());
        }
        Pose defaultStart = (autoEnd != null) ? autoEnd : parkPose;
        String defaultName = (autoEnd != null) ? "WHERE THE AUTO STOPPED" : "PARK SPOT";

        boolean startAtShot1 = false;
        follower.setPose(defaultStart);
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

        // INIT loop: choose and show the start position
        while (!isStarted() && !isStopRequested()) {

            boolean before = startAtShot1;
            if (gamepad1.dpad_left) {
                startAtShot1 = true;
            }
            if (gamepad1.dpad_right) {
                startAtShot1 = false;
            }
            if (before != startAtShot1) {
                follower.setPose(startAtShot1 ? shot1Spot : defaultStart);
            }
            follower.update();

            Pose now = follower.pose();
            telemetry.addData("Status", "Initialized");
            telemetry.addData("START SPOT", startAtShot1 ? "SHOT 1 spot (0, 0, 0)" : defaultName);
            telemetry.addData("Robot position now", "(%.1f, %.1f, %.0f°)",
                    now.x(), now.y(), Math.toDegrees(now.heading()));
            telemetry.addData("Robot on the shot 1 spot?", "D-pad Left");
            telemetry.addData("Back to " + defaultName.toLowerCase() + "?", "D-pad Right");
            telemetry.update();
        }

        if (isStopRequested()) {
            return;
        }

        // Set the start position again now that play is pressed, and check
        // that it stuck (the odometry chip can finish resetting late).
        setStartPose(startAtShot1 ? shot1Spot : defaultStart);

        // The auto's position has now been used up
        Position_Storage.fromAuto = false;


        // =====================================================
        // MAIN LOOP
        // =====================================================

        while (opModeIsActive()) {

            // =================================================
            // WAYPOINT CONTROLS - GAMEPAD 1
            // =================================================

            // D-pad Up = shot 1 spot
            if (gamepad1.dpad_up && !lastDpadUp1) {
                goToSpot(false);
            }

            // D-pad Down = shot 2 spot
            if (gamepad1.dpad_down && !lastDpadDown1) {
                goToSpot(true);
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

                if (stage == 1) {
                    // Driving the lane; ends STOPPED in front of the shot spot
                    if (!follower.isBusy()) {
                        stage = 2;
                        stageTimer.reset();
                    }
                } else if (stage == 2) {
                    // Let the turn finish completely before the straight push
                    if (stageTimer.seconds() > settleSeconds) {
                        startStraightIn();
                    }
                } else if (stage == 3) {
                    // Slow straight push; stop pushing after a short time
                    // even if something blocks it an inch or two short
                    if (!follower.isBusy() || stageTimer.seconds() > STRAIGHT_IN_LIMIT_S) {
                        Pose q = follower.pose();
                        Pose t = goingToShot2 ? shot2Spot : shot1Spot;
                        follower.hold(new Pose(q.x(), q.y(), t.heading()));
                        finishWaypoint("Arrived");
                    }
                }

                if (waypointActive && waypointTimer.seconds() > WAYPOINT_TIMEOUT_S) {
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
                    !waypointActive ? "Manual"
                            : stage == 1 ? "AUTO: driving the lane"
                            : stage == 2 ? "AUTO: lining up"
                            : "AUTO: straight in"
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

            telemetry.addData("D-pad Up", "shot 1 spot");
            telemetry.addData("D-pad Down", "shot 2 spot");

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

    // Same lane points the auto uses:
    //   shot 1 approach -> loading back-off -> beside the stand
    //   -> just past the stand -> line-up spot in front of shot 2
    private void buildLane() {

        Pose shot1Approach = backOff(shot1Spot, 12);
        Pose loadingBackOff = backOff(loadingSpot, 20);

        double dx = shot2Spot.x() - besideStand.x();
        double dy = shot2Spot.y() - besideStand.y();
        double len = Math.hypot(dx, dy);
        Pose clearOfStand = new Pose(
                besideStand.x() + 10 * dx / len,
                besideStand.y() + 10 * dy / len,
                besideStand.heading());

        Pose lineUp = backOff(shot2Spot, 18);
        shot2Stop = backOff(shot2Spot, 2);

        lane = new Pose[]{shot1Approach, loadingBackOff, besideStand, clearOfStand, lineUp};
    }

    // A pose moved straight back along the way it faces (same heading)
    private static Pose backOff(Pose p, double inches) {
        return new Pose(
                p.x() - inches * Math.cos(p.heading()),
                p.y() - inches * Math.sin(p.heading()),
                p.heading());
    }

    private static double shortTurn(double from, double to) {
        double t = to - from;
        return Math.atan2(Math.sin(t), Math.cos(t));
    }

    private static double dist(Pose a, Pose b) {
        return Math.hypot(a.x() - b.x(), a.y() - b.y());
    }

    private Pose currentPose() {
        Pose p = follower.pose();
        return new Pose(p.x(), p.y(), p.heading());
    }

    // Set the start position and make sure it actually took
    private void setStartPose(Pose start) {
        for (int i = 0; i < 10 && opModeIsActive(); i++) {
            follower.setPose(start);
            sleep(50);
            follower.update();
            Pose q = follower.pose();
            if (Math.hypot(q.x() - start.x(), q.y() - start.y()) < 1.0
                    && Math.abs(shortTurn(q.heading(), start.heading())) < Math.toRadians(2)) {
                return;
            }
        }
    }

    // D-pad Up (shot 1) or D-pad Down (shot 2)
    private void goToSpot(boolean toShot2) {

        driverHasControl = true;

        Pose cur = currentPose();
        Pose target = toShot2 ? shot2Spot : shot1Spot;

        goingToShot2 = toShot2;
        activeWaypoint = target;
        targetName = toShot2 ? "Shot 2" : "Shot 1";
        waypointActive = true;
        waypointTimer.reset();

        // Already there: just hold it
        if (dist(cur, target) < 3.0) {
            follower.hold(target);
            finishWaypoint("Arrived");
            return;
        }

        // Join the lane at the closest lane point, then follow the lane
        // toward the chosen shot spot
        int nearest = 0;
        for (int i = 1; i < lane.length; i++) {
            if (dist(cur, lane[i]) < dist(cur, lane[nearest])) {
                nearest = i;
            }
        }
        int last = toShot2 ? lane.length - 1 : 0;
        int step = toShot2 ? 1 : -1;

        ArrayList<Pose> pts = new ArrayList<>();
        for (int i = nearest; ; i += step) {
            pts.add(lane[i]);
            if (i == last) {
                break;
            }
        }

        ArrayList<Path> legs = new ArrayList<>();
        Pose from = cur;
        double h = cur.heading();
        int k = 0;

        if (dist(cur, pts.get(0)) < 2.0) {
            // Already on the lane point; start from here
            k = 1;
        } else {
            // Leg onto the lane
            Pose entry = pts.get(0);
            double hEntry;
            if (pts.size() == 1) {
                // Joining right at the line-up spot: turn to the final heading
                hEntry = h + shortTurn(h, entry.heading());
            } else if (nearest == 0 || nearest == 1 || nearest == lane.length - 1) {
                // Near a wall, loading, or the flower:
                // back straight out first, no turning
                hEntry = h;
            } else {
                // Turn to how the robot drives the next part of the lane
                hEntry = h + shortTurn(h, pts.get(1).heading());
            }
            legs.add(Paths.line(from, entry).linear(h, hEntry));
            from = entry;
            h = hEntry;
            k = 1;
        }

        for (; k < pts.size(); k++) {
            Pose b = pts.get(k);
            double hb = h + shortTurn(h, b.heading());
            legs.add(Paths.line(from, b).linear(h, hb));
            from = b;
            h = hb;
        }

        if (legs.isEmpty()) {
            // Sitting on the line-up spot: turn there, then push in
            follower.hold(pts.get(pts.size() - 1));
            settleSeconds = 1.0;
            stage = 2;
            stageTimer.reset();
            return;
        }

        settleSeconds = 0.3;
        follower.follow(Paths.path(legs.toArray(new Path[0]))
                .with(Constants.foresightConfig.maxPathSpeed.at(WAYPOINT_SPEED)));
        stage = 1;
    }

    // Slow, straight push from the line-up spot into the shot spot
    private void startStraightIn() {

        Pose from = currentPose();
        Pose to = goingToShot2 ? shot2Stop : shot1Spot;

        if (dist(from, to) < 1.0) {
            follower.hold(to);
            finishWaypoint("Arrived");
            return;
        }

        follower.follow(Paths
                .line(from, to)
                .constant(to)
                .with(Constants.foresightConfig.maxPathSpeed.at(STRAIGHT_IN_SPEED)));
        stage = 3;
        stageTimer.reset();
    }

    private void finishWaypoint(String status) {

        waypointActive = false;
        stage = 0;
        activeWaypoint = null;
        targetName = status;
    }

    private void cancelWaypoint(String reason) {

        follower.stop();
        finishWaypoint(reason);
    }
}
