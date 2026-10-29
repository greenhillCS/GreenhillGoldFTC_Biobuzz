package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler; // Import the scheduler
import static com.pedropathing.ivy.Scheduler.schedule; // Required to run paths
import static com.pedropathing.ivy.pedro.PedroCommands.follow; // Required to target the follower
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "PathARed", group = "Autonomous")
public class PathARed extends LinearOpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Step control variables
    private int currentStep = 2;
    private final ElapsedTime actionTimer = new ElapsedTime();

    // Robot hardware variables
    private DcMotor shooter;
    private DcMotor intake;
    private Servo gate;

    // Servo Position Constants
    private final double GATE_OPEN = 1.0;
    private final double GATE_CLOSED = 0.0;

    // Field Coordinates (Updated with clearance waypoint)
    private final Pose startAndScorePose = poseFactory.of(56.0, 8.0, 90);
    private final Pose wallClearancePose = poseFactory.of(56.0, 24.0, 90);
    private final Pose intakePose = poseFactory.of(56, 56, 270);

    // NEW: Clearance point to steer clear of field obstacles/pollen elements on the way to parking
    // Adjust x: 30.0 and y: 100.0 based on where your exact field hazard lies
    private final Pose clearancePose = poseFactory.of(30.0, 50, 90);
    private final Pose parkPose = poseFactory.of(10.0, 64, 90);

    @Override
    public void runOpMode() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startAndScorePose);
        follower.update();

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        gate = hardwareMap.get(Servo.class, "gateServo");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gate.setPosition(GATE_CLOSED);

        waitForStart();
        if (isStopRequested()) return;

        actionTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("Step", currentStep);
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.update();

            // STEP 2: Shoot from wall setup
            if (currentStep == 2) {
                gate.setPosition(GATE_OPEN);
                shooter.setPower(1.0);

                if (actionTimer.seconds() >= 3.0) {
                    shooter.setPower(0.0);
                    gate.setPosition(GATE_CLOSED);

                    schedule(follow(follower, path1()));
                    currentStep = 3;
                }
            }

            // STEP 3: Clear wall, transition to intake location
            if (currentStep == 3 && !follower.isBusy()) {
                schedule(follow(follower, path2()));
                intake.setPower(1.0);
                currentStep = 4;
            }

            // STEP 4: Arrive at intake spot
            if (currentStep == 4 && !follower.isBusy()) {
                actionTimer.reset();
                currentStep = 5;
            }

            // STEP 5: Intake elements for 2 seconds, then transition to clearance waypoint
            if (currentStep == 5) {
                intake.setPower(1.0);
                if (actionTimer.seconds() >= 2.0) {
                    intake.setPower(0.0);
                    schedule(follow(follower, path3())); // MODIFIED: Now runs to the Clearance Waypoint
                    currentStep = 6;
                }
            }

            // NEW STEP 6: Wait for clearance maneuver to finish, then proceed directly to parking
            if (currentStep == 6 && !follower.isBusy()) {
                schedule(follow(follower, path4())); // NEW: Travel safely from clearance position to parking
                currentStep = 7;
            }

            // STEP 7: Wait until the robot finishes parking, then stop running
            if (currentStep == 7 && !follower.isBusy()) {
                currentStep = 8;
            }
        }
    }

    // Path 1: Drive straight forward off the wall
    public Path path1() {
        return Paths.line(startAndScorePose, wallClearancePose).linear(startAndScorePose, wallClearancePose);
    }

    // Path 2: Drive to the intake position
    public Path path2() {
        return Paths.line(wallClearancePose, intakePose).linear(wallClearancePose, intakePose);
    }

    // MODIFIED Path 3: Drive out of the intake zone to the safe clearance waypoint
    public Path path3() {
        return Paths.line(intakePose, clearancePose).linear(intakePose, clearancePose);
    }

    // NEW Path 4: Final stretch from clearance waypoint to the parking spot
    public Path path4() {
        return Paths.line(clearancePose, parkPose).linear(clearancePose, parkPose);
    }
}
