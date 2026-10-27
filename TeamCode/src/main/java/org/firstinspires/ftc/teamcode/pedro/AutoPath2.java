package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto Path 2", group = "Autonomous")
public class AutoPath2 extends LinearOpMode {

    private Follower follower;

    private DcMotorEx shooter;
    private DcMotorEx intake;
    private Servo gateServo;

    private final double shooterVelocity = 1700.0;
    private final double intakeVelocity = 800.0;

    private static final double GATE_CLOSED = 0.70;
    private static final double GATE_OPEN = 0.40;

    private final ElapsedTime runtime = new ElapsedTime();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58.7125, 126.8962, -90);
    private final Pose path1 = poseFactory.of(58.7125, 126.8962, -90);
    private final Pose point2 = poseFactory.of(58.7125, 12.4577, 90);
    private final Pose point3Start = poseFactory.of(58.7125, 12.4577, 0);
    private final Pose point3 = poseFactory.of(8.5232, 96.5982, 360);
    private final Pose point3Control1 = poseFactory.of(4.0998, 47.3674, 0);

    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3())
        );
    }

    @Override
    public void runOpMode() {

        Scheduler.reset();

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setVelocity(0);
        intake.setVelocity(0);

        gateServo.setPosition(GATE_CLOSED);

        follower = Constants.create(hardwareMap);

        follower.setPose(start);
        follower.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        runtime.reset();

        intake.setVelocity(intakeVelocity);

        schedule(autoRoutine());

        while (opModeIsActive()) {

            follower.update();
            Scheduler.execute();

            double time = runtime.seconds();

            intake.setVelocity(intakeVelocity);

            if (time >= 15.0) {

                shooter.setVelocity(shooterVelocity);
                gateServo.setPosition(GATE_OPEN);

            } else {

                shooter.setVelocity(0);
                gateServo.setPosition(GATE_CLOSED);
            }

            telemetry.addData("Time", "%.1f", time);
            telemetry.addData("Shooter Target", "%.0f", time >= 15.0 ? shooterVelocity : 0);
            telemetry.addData("Shooter Actual", "%.0f", shooter.getVelocity());
            telemetry.addData("Intake Actual", "%.0f", intake.getVelocity());
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

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

        shooter.setVelocity(0);
        intake.setVelocity(0);
        gateServo.setPosition(GATE_CLOSED);
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).reverseTangent();
    }

    public Path path3() {
        return Paths.curve(
                point3Start,
                point3Control1,
                point3
        ).linear(point3Start, point3);
    }
}