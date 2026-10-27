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

@Autonomous(name = "Auto Path 1", group = "Autonomous")
public class AutoPath1 extends LinearOpMode {

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

    private final Pose start = poseFactory.of(58.6952, 6.8017, 90);
    private final Pose point1 = poseFactory.of(58.6392, 12.7903, 90.5361);
    private final Pose path1Start = poseFactory.of(58.6392, 12.7903, 90);
    private final Pose path1 = poseFactory.of(8.355, 8.4462, -90);
    private final Pose point3Start = poseFactory.of(8.355, 8.4462, -90);
    private final Pose point3 = poseFactory.of(29.7, 8.4462, -90);
    private final Pose point4Start = poseFactory.of(29.7, 8.4462, 0);
    private final Pose point4 = poseFactory.of(58.6615, 128.3834, -90);
    private final Pose point4Control1 = poseFactory.of(21.7738, 123.643, 0);
    private final Pose point5Start = poseFactory.of(58.6615, 128.3834, 0);
    private final Pose point5 = poseFactory.of(6.6602, 116.0071, 0);
    private final Pose point5Control1 = poseFactory.of(25.8651, 119.4696, 0);
    private final Pose point6 = poseFactory.of(64.8219, 90.1316, -23.9837);

    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path1_2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6())
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

        shooter.setVelocity(shooterVelocity);
        intake.setVelocity(intakeVelocity);

        schedule(autoRoutine());

        while (opModeIsActive()) {

            follower.update();
            Scheduler.execute();

            double time = runtime.seconds();

            shooter.setVelocity(shooterVelocity);

            if (time < 25.0) {
                intake.setVelocity(intakeVelocity);
            } else {
                intake.setVelocity(0);
            }

            if ((time >= 1.0 && time < 2.0) ||
                    (time >= 17.0 && time < 18.0)) {

                gateServo.setPosition(GATE_OPEN);

            } else {

                gateServo.setPosition(GATE_CLOSED);
            }

            telemetry.addData("Time", "%.1f", time);
            telemetry.addData("Shooter Target", "%.0f", shooterVelocity);
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
        return Paths.line(start, point1).tangent();
    }

    public Path path1_2() {
        return Paths.line(path1Start, path1).linear(path1Start, path1);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.curve(
                point4Start,
                point4Control1,
                point4
        ).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.curve(
                point5Start,
                point5Control1,
                point5
        ).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point5, point6).tangent();
    }
}
