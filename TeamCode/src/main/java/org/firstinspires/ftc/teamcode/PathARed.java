package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Position_Storage;

@Autonomous(name = "PathARed", group = "Autonomous")
public class PathARed extends LinearOpMode {

    private Follower follower;

    private DcMotorEx shooter;
    private DcMotorEx intake;
    private Servo ramp;

    private static final double SHOOTER_VELOCITY = 1200.0;    // shot 1, from the start
    private static final double SHOOTER_VELOCITY_2 = 1200.0;  // shot 2, from the shooting spot
    private static final double INTAKE_VELOCITY = 800.0;      // same as TeleOp
    private static final int SPIN_UP_MS = 5000;   // shooter spins this long before shot 1
    private static final int FIRE_MS = 5000;      // ramp stays up this long for each shot
    private static final int LOADING_WAIT_MS = 2000;          // time parked at loading to grab balls

    // Path speed while testing (fraction of max). Raise to 1.0 once it behaves.
    private static final double AUTO_SPEED = 0.5;

    private static final double RAMP_DOWN = 0.22;   // lowered so balls don't slip under
    private static final double RAMP_UP = 0.45;

    // =====================================================
    // POSES: SAME MAP AS YOUR TELEOP
    // =====================================================
    // (0, 0, 0) is your usual TeleOp starting spot and direction.
    // Place the robot there exactly like you do for TeleOp.
    // The other three were recorded by driving there in TeleOp.
    // If you re-record them, paste the new telemetry numbers here.

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start =
            poseFactory.of(0, 0, 0);

    private final Pose loading =
            poseFactory.of(3.03, -47.37, 5.5);

    // Re-recorded after the Pinpoint offset fix
    private final Pose shootingSpot =
            poseFactory.of(-114.87, -2.84, 184.4);

    // Re-recorded after the Pinpoint offset fix
    private final Pose park =
            poseFactory.of(-106.79, -51.95, 93);

    // How far the arch to loading bows away from the wall, in inches.
    // Bigger = backs farther off the wall before sliding over.
    private static final double ARCH_HEIGHT = 24.0;

    // Arch from the start to loading (the upside-down parabola).
    // archBack1 is straight behind the start, archBack2 is straight behind
    // loading, so the robot backs STRAIGHT off the wall, slides over, and
    // drives STRAIGHT into the loading space intake first. It keeps facing
    // the wall the whole time, so it never turns while touching the wall.
    private final Pose archBack1 = new Pose(
            start.x() - ARCH_HEIGHT * Math.cos(start.heading()),
            start.y() - ARCH_HEIGHT * Math.sin(start.heading()),
            start.heading());

    private final Pose archBack2 = new Pose(
            loading.x() - ARCH_HEIGHT * Math.cos(loading.heading()),
            loading.y() - ARCH_HEIGHT * Math.sin(loading.heading()),
            loading.heading());

    // Leaving loading: back straight off the wall first (20 in), then
    // head for the shooting spot along your orange line.
    private final Pose loadingBackOff = new Pose(
            loading.x() - 20 * Math.cos(loading.heading()),
            loading.y() - 20 * Math.sin(loading.heading()),
            loading.heading());

    // =====================================================
    // ROUTE FROM LOADING TO SHOT 2
    // =====================================================
    // Beside the goal stand, intake still facing loading (recorded in TeleOp).
    private final Pose besideStand =
            poseFactory.of(-73.31, -30.73, 342);

    // After passing the stand, drive this far straight (no turning) to get
    // clear of the stand's leg before turning around.
    private static final double CLEAR_STAND = 10.0;

    // The last part into shot 2 is this long and perfectly straight, with
    // the turn already finished, so the corners never swing into the flower.
    private static final double STRAIGHT_IN = 18.0;

    // Stop this many inches short of the shot 2 spot.
    // Raise it if the robot bumps the flower.
    private static final double STOP_SHORT = 2.0;

    // Speed for the final straight push into shot 2 (slow and controlled)
    private static final double STRAIGHT_IN_SPEED = 0.3;


    // Unit direction from beside the stand to the shot 2 spot
    private double[] standToShotDir() {
        double dx = shootingSpot.x() - besideStand.x();
        double dy = shootingSpot.y() - besideStand.y();
        double len = Math.hypot(dx, dy);
        return new double[]{dx / len, dy / len};
    }

    // Clear of the stand, still facing loading
    private Pose clearOfStand() {
        double[] d = standToShotDir();
        return new Pose(
                besideStand.x() + CLEAR_STAND * d[0],
                besideStand.y() + CLEAR_STAND * d[1],
                besideStand.heading());
    }

    // Direction the intake points at the shot 2 spot. The last part comes in
    // straight along this line (no sideways movement), so the robot's side
    // never sweeps past the flower that sits next to the shot 2 spot.
    private double[] shotFacing() {
        return new double[]{
                Math.cos(shootingSpot.heading()),
                Math.sin(shootingSpot.heading())};
    }

    // Lined up straight in front of the shot 2 spot, already turned around
    private Pose lineUp() {
        double[] u = shotFacing();
        return new Pose(
                shootingSpot.x() - STRAIGHT_IN * u[0],
                shootingSpot.y() - STRAIGHT_IN * u[1],
                shootingSpot.heading());
    }

    private Pose shot2Stop() {
        double[] u = shotFacing();
        return new Pose(
                shootingSpot.x() - STOP_SHORT * u[0],
                shootingSpot.y() - STOP_SHORT * u[1],
                shootingSpot.heading());
    }

    // Shorter-way turn from one heading to another (radians)
    private static double shortTurn(double from, double to) {
        double t = to - from;
        return Math.atan2(Math.sin(t), Math.cos(t));
    }

    // =====================================================
    // AUTONOMOUS ROUTINE
    // =====================================================

    // Shot 1 and loading
    private Command firstHalf() {

        return sequential(

                // From the very first second: intake ON (pushes balls to the
                // ramp) and shooter ON (spins up). Both stay on.
                instant(() -> {
                    intake.setVelocity(INTAKE_VELOCITY);
                    shooter.setVelocity(SHOOTER_VELOCITY);
                }),

                // Let the shooter spin up
                waitMs(SPIN_UP_MS),

                // ---------- SHOT 1 (from the start) ----------

                instant(() ->
                        ramp.setPosition(RAMP_UP)
                ),
                waitMs(FIRE_MS),
                instant(() ->
                        ramp.setPosition(RAMP_DOWN)
                ),

                // Shooter keeps spinning for shot 2
                instant(() ->
                        shooter.setVelocity(SHOOTER_VELOCITY_2)
                ),

                waitMs(500),

                // ---------- LOADING ----------

                // Arch over to loading: back off the wall, slide over,
                // then drive straight into the loading space intake first
                instant(() -> follower.follow(toLoading().with(Constants.foresightConfig.maxPathSpeed.at(AUTO_SPEED)))),
                waitMs(3000),
                waitMs(LOADING_WAIT_MS)
        );
    }

    public Command autoRoutine() {

        return sequential(

                firstHalf(),

                // ---------- SHOT 2 (shooting spot) ----------

                // Drive to the line-up spot 18" in front of shot 2 and STOP there,
                // with the turn-around fully finished
                follow(follower, toLineUp().with(Constants.foresightConfig.maxPathSpeed.at(AUTO_SPEED))),
                waitMs(300),

                // Slow, straight push into shot 2 (no turning, no sliding)
                // Push into shot 2, but give up after 2 seconds if something stops it
                // a few inches short (it shoots from there instead of waiting forever)
                instant(() -> follower.follow(straightIn().with(Constants.foresightConfig.maxPathSpeed.at(STRAIGHT_IN_SPEED)))),
                waitMs(2000),

                // Short pause before the ramp lifts
                waitMs(500),

                instant(() ->
                        ramp.setPosition(RAMP_UP)
                ),
                waitMs(FIRE_MS),
                instant(() ->
                        ramp.setPosition(RAMP_DOWN)
                ),
                instant(() ->
                        shooter.setVelocity(0)
                ),

                // ---------- PARK ----------

                follow(follower, toPark().with(Constants.foresightConfig.maxPathSpeed.at(AUTO_SPEED))),

                // Shooter off, ramp down. Intake stays on until the auto ends.
                instant(() -> {
                    shooter.setVelocity(0);
                    ramp.setPosition(RAMP_DOWN);
                })
        );
    }

    // =====================================================
    // RUN OPMODE
    // =====================================================

    @Override
    public void runOpMode() {

        Scheduler.reset();

        // Hardware setup
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        ramp = hardwareMap.get(Servo.class, "ramp");

        shooter.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Same shooter PIDF as TeleOp
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new com.qualcomm.robotcore.hardware.PIDFCoefficients(100, 0, 0.01, 12));

        shooter.setVelocity(0);
        intake.setVelocity(0);
        ramp.setPosition(RAMP_DOWN);

        // Pedro initialization
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Place robot", "Same spot and direction as TeleOp (0, 0, 0)");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        schedule(autoRoutine());

        while (opModeIsActive()) {

            follower.update();
            Scheduler.execute();

            // Hand the position to TeleOp so (0, 0, 0) stays the shot 1 spot
            Position_Storage.pos = follower.pose();
            Position_Storage.fromAuto = true;

            // Telemetry
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData(
                    "heading",
                    Math.toDegrees(follower.pose().heading())
            );

            telemetry.addData(
                    "Shooter Velocity",
                    shooter.getVelocity()
            );

            if (follower.currentPath() != null) {

                telemetry.addData(
                        "Current path distance remaining",
                        follower.distanceToEndpoint()
                );

                telemetry.addData(
                        "Path number",
                        follower.pathIndex()
                );
            }

            telemetry.update();
        }

        // =====================================================
        // STOP
        // =====================================================

        shooter.setVelocity(0);
        intake.setVelocity(0);
        ramp.setPosition(RAMP_DOWN);
        follower.stop();
    }

    // =====================================================
    // PATHS
    // =====================================================

    // The arch to loading. Heading barely changes (start heading to the
    // recorded loading heading), so the intake faces the wall the whole time.
    public Path toLoading() {

        return Paths
                .curve(start, archBack1, archBack2, loading)
                .linear(start, loading);
    }

    // Loading -> beside the stand (still facing loading) -> a little past the
    // stand -> turn around while lining up. Ends STOPPED at the line-up spot.
    public Path toLineUp() {

        Pose clear = clearOfStand();
        Pose lined = lineUp();

        double h0 = loading.heading();
        double h1 = h0 + shortTurn(h0, besideStand.heading());
        double h2 = h1 + shortTurn(h1, shootingSpot.heading());

        // Back out of loading to beside the stand, barely turning
        Path toStand = Paths
                .curve(loading, loadingBackOff, besideStand)
                .linear(h0, h1);

        // Straight away from the stand's leg, no turning
        Path pastStand = Paths
                .line(besideStand, clear)
                .linear(h1, h1);

        // Turn around (the shorter way) while lining up
        Path turnAround = Paths
                .line(clear, lined)
                .linear(h1, h2);

        return Paths.path(toStand, pastStand, turnAround);
    }

    // Line-up spot -> shot 2: straight, no turning
    public Path straightIn() {

        return Paths
                .line(lineUp(), shot2Stop())
                .constant(shootingSpot);
    }

    // Shot 2 -> park. Backs out EXACTLY the way it came in (straight back to
    // the line-up spot, then back to just past the stand) without turning,
    // and only turns toward park after that, far from the flower.
    public Path toPark() {

        Pose stop = shot2Stop();
        Pose lined = lineUp();
        Pose clear = clearOfStand();

        double hs = shootingSpot.heading();
        double hp = hs + shortTurn(hs, park.heading());

        Path backToLineUp = Paths
                .line(stop, lined)
                .constant(shootingSpot);

        Path backToStand = Paths
                .line(lined, clear)
                .constant(shootingSpot);

        Path toParkSpot = Paths
                .line(clear, park)
                .linear(hs, hp);

        return Paths.path(backToLineUp, backToStand, toParkSpot);
    }
}
