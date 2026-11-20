
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


@Autonomous(name = "PathABlue", group = "Autonomous")
public class PathABlue extends LinearOpMode {

    private Follower follower;

    private DcMotorEx shooter;
    private DcMotorEx intake;
    private Servo ramp;

    private static final double SHOOTER_VELOCITY = 1200.0;

    // Corrected ramp positions
    private static final double RAMP_DOWN = 0.25;
    private static final double RAMP_UP = 0.45;


    // =====================================================
    // PEDRO PATHING POSES
    // =====================================================

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start =
            poseFactory.of(63.5, 132.7, 90);

    private final Pose point1 =
            poseFactory.of(59.1798, 34.4101, 90);

    private final Pose point2 =
            poseFactory.of(15, 91.7063, 90);

    private final Pose point2Control1 =
            poseFactory.of(19.2657, 34.256, 0);


    // =====================================================
    // AUTONOMOUS ROUTINE
    // =====================================================

    public Command autoRoutine() {

        return sequential(

                // Wait 5 seconds
                waitMs(5000),

                // Start shooter
                instant(() ->
                        shooter.setVelocity(SHOOTER_VELOCITY)
                ),

                // Allow shooter to spin up
                waitMs(5000),

                // Raise ramp
                instant(() ->
                        ramp.setPosition(RAMP_UP)
                ),

                // Keep ramp raised for 2 seconds
                waitMs(2000),

                // Lower ramp
                instant(() ->
                        ramp.setPosition(RAMP_DOWN)
                ),

                // Stop shooter after firing
                instant(() ->
                        shooter.setVelocity(0)
                ),

                // Wait before driving
                waitMs(500),

                // Follow first path
                follow(follower, path1()),

                // Follow second path
                follow(follower, path2()),

                // Stop mechanisms after movement
                instant(() -> {
                    shooter.setVelocity(0);
                    intake.setVelocity(0);
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

        shooter.setVelocity(0);
        intake.setVelocity(0);
        ramp.setPosition(RAMP_DOWN);


        // Pedro initialization
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        schedule(autoRoutine());


        while (opModeIsActive()) {

            follower.update();
            Scheduler.execute();

            // Save position for TeleOp
            Position_Storage.pos = follower.pose();

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
    // PATH 1
    // =====================================================

    public Path path1() {

        return Paths
                .line(start, point1)
                .constant(point1);
    }


    // =====================================================
    // PATH 2
    // =====================================================

    public Path path2() {

        return Paths
                .curve(point1, point2Control1, point2)
                .constant(point2);
    }
}
