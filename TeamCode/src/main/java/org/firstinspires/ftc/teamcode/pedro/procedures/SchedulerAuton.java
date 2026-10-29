
package org.firstinspires.ftc.teamcode.pedro.procedures;

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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "SchedulerAuton", group = "Autonomous")
public class SchedulerAuton extends LinearOpMode {

    private Follower follower;

    private DcMotor shooter;
    private DcMotor intake;
    private Servo gateServo;

    private static final double SHOOTER_POWER = 0.50;
    private static final double INTAKE_POWER = 1.0;
    private static final double GATE_CLOSED = 0.65;
    private static final double GATE_OPEN = 0.40;

    // Pedro Pathing Poses
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(52.7319, 9.7271, 270);
    private final Pose point1 = poseFactory.of(24.2271, 9.2729, 180);
    private final Pose point1Control1 = poseFactory.of(49.7408, 20.0226, 0);

    private final Pose point2 = poseFactory.of(9.2729, 9.2729, 180);

    private final Pose point3Start = poseFactory.of(9.2729, 9.2729, 180);
    private final Pose point3 = poseFactory.of(61, 120, 90);
    private final Pose point3Control1 = poseFactory.of(40.2087, 26.4583, 0);
    private final Pose point3Control2 = poseFactory.of(10.1364, 121.3555, 0);

    private final Pose point4 = poseFactory.of(61, 131.5, 90);

    private final Pose point5Start = poseFactory.of(61, 131.5, 90);
    private final Pose point5 = poseFactory.of(10.553, 121.2961, 0);
    private final Pose point5Control1 = poseFactory.of(61.0915, 110.2994, 0);
    private final Pose point5Control2 = poseFactory.of(25.1581, 120.5923, 0);
    private final Pose point5Control3 = poseFactory.of(34.5963, 115.5136, 0);

    // Mechanism controls

    private void shooterOn() {
        shooter.setPower(SHOOTER_POWER);
    }

    private void gateOpen() {
        gateServo.setPosition(GATE_OPEN);
    }

    private void gateClose() {
        gateServo.setPosition(GATE_CLOSED);
    }

    private void intakeOn() {
        intake.setPower(INTAKE_POWER);
    }

    private void stopMechanisms() {
        shooter.setPower(0);
        intake.setPower(0);
        gateClose();
    }

    private void off() {
        shooter.setPower(0);
        intake.setPower(0);
        gateClose();
    }

    // Autonomous sequence

    public Command autoRoutine() {
        return sequential(

                // First shooting sequence
                instant(() -> shooterOn()),
                waitMs(5000),

                instant(() -> gateOpen()),
                waitMs(2000),
                instant(() -> gateClose()),

                // Intake and paths 1-2
                waitMs(500),
                instant(() -> intakeOn()),
                follow(follower, path1()),
                follow(follower, path2()),

                // Paths 3-4
                waitMs(1000),
                follow(follower, path3()),
                follow(follower, path4()),

                // Second shooting sequence
                instant(() -> shooterOn()),
                waitMs(2000),

                instant(() -> gateOpen()),
                waitMs(2000),
                instant(() -> gateClose()),

                waitMs(500),

                // Final path
                instant(() -> stopMechanisms()),
                follow(follower, path5()),

                // Stop everything
                instant(() -> off())
        );
    }

    @Override
    public void runOpMode() {

        Scheduler.reset();

        // Initialize Pedro Pathing
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        // Initialize hardware
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        gateServo = hardwareMap.get(Servo.class, "gate");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        off();

        telemetry.addLine("Autonomous Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            off();
            return;
        }

        Scheduler.reset();
        schedule(autoRoutine());

        try {
            while (opModeIsActive()) {

                follower.update();
                Scheduler.execute();

                // Telemetry
                telemetry.addData("x", follower.pose().x());
                telemetry.addData("y", follower.pose().y());
                telemetry.addData("heading",
                        Math.toDegrees(follower.pose().heading()));

                if (follower.currentPath() != null) {
                    telemetry.addData(
                            "Current path distance remaining",
                            follower.distanceToEndpoint()
                    );
                    telemetry.addData("Path number", follower.pathIndex());
                }

                telemetry.update();
                idle();
            }
        } finally {
            off();
            Scheduler.reset();
        }
    }

    // Paths

    public Path path1() {
        return Paths.curve(start, point1Control1, point1)
                .linear(start, point1);
    }

    public Path path2() {
        return Paths.line(point1, point2).tangent();
    }

    public Path path3() {
        return Paths.curve(
                point3Start,
                point3Control1,
                point3Control2,
                point3
        ).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.line(point3, point4).constant(point4);
    }

    public Path path5() {
        return Paths.curve(
                point5Start,
                point5Control1,
                point5Control2,
                point5Control3,
                point5
        ).linear(point5Start, point5);
    }
}
