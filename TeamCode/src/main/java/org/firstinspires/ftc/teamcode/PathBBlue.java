
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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Position_Storage;


@Autonomous(name = "PathBBlue", group = "Autonomous")
public class PathBBlue extends LinearOpMode {

    private Follower follower;

    private DcMotorEx shooter;
    private DcMotorEx intake;
    private Servo ramp;

    private static final double SHOOTER_VELOCITY = 1200.0;

    private static final double RAMP_DOWN = 0.3;
    private static final double RAMP_UP = 0.45;


    // =====================================================
    // PEDRO PATHING POSES
    // =====================================================

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start =
            poseFactory.of(58.7125, 126.8962, -90);

    private final Pose point2 =
            poseFactory.of(58.7125, 12.4577, 90);

    private final Pose point3 =
            poseFactory.of(10.7395, 96.8753, 360);

    private final Pose point3Control1 =
            poseFactory.of(4.0998, 47.3674, 0);


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

                // Follow first movement path
                follow(follower, path2()),

                // Follow second movement path
                follow(follower, path3()),

                // Turn mechanisms off
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

        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(100, 0, 0.015, 12.5)
        );

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
    // PATH 2
    // =====================================================

    public Path path2() {

        return Paths
                .line(start, point2)
                .reverseTangent();
    }


    // =====================================================
    // PATH 3
    // =====================================================

    public Path path3() {

        return Paths
                .curve(point2, point3Control1, point3)
                .linear(point2, point3);
    }
}
