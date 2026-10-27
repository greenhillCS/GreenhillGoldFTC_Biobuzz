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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "PathARed", group = "Autonomous")
public class PathARed extends LinearOpMode {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Step control starts immediately on Step 2 (Shooting from wall)
    private int currentStep = 2;
    private final ElapsedTime actionTimer = new ElapsedTime();

    // Robot hardware motor variables
    private DcMotor shooter;
    private DcMotor intake;

    // Field Coordinates updated with your exact adjusted points
    private final Pose startAndScorePose = poseFactory.of(56.0, 8.0, -90);
    private final Pose wallClearancePose = poseFactory.of(56.0, 24.0, -90); // Forward step to avoid hitting wall
    private final Pose intakePose        = poseFactory.of(10.0, 9.0, 180);
    private final Pose parkPose          = poseFactory.of(10.0, 190.0, -90);

    @Override
    public void runOpMode() {
        Scheduler.reset(); // Reset Ivy scheduler on init

        // Initialize the drivetrain follower using your team's Constants
        follower = Constants.create(hardwareMap);
        follower.setPose(startAndScorePose);
        follower.update();

        // Initialize the mechanism motors from the hardware map configuration
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Set motors to brake mode so they stop instantly when power is set to 0
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (isStopRequested()) return;

        // Reset our timer immediately when the match starts so Step 2 counts cleanly
        actionTimer.reset();

        // Main simple loop that runs during the 30-second autonomous period
        while (opModeIsActive() && !isStopRequested()) {
            follower.update(); // Keep the drivetrain moving and tracking position
            Scheduler.execute(); // Tell Ivy to process active commands

            // Telemetry reporting for troubleshooting
            telemetry.addData("Step", currentStep);
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.update();

            // STEP 2: Hold still and shoot immediately from the starting wall position for 3 seconds
            if (currentStep == 2) {
                shooter.setPower(1.0); // Run shooter motor at full power

                if (actionTimer.seconds() >= 3.0) {
                    shooter.setPower(0.0); // Shut off shooter motor completely
                    schedule(follow(follower, path1())); // Drive straight forward away from wall
                    currentStep = 3;
                }
            }

            // STEP 3: Wait until the robot finishes moving forward away from the wall
            if (currentStep == 3 && !follower.isBusy()) {
                schedule(follow(follower, path2())); // Drive sideways to the intake location
                intake.setPower(1.0); // START INTAKE RUNNING ON THE WAY!
                currentStep = 4;
            }

            // STEP 4: Wait until the robot arrives at the intake spot
            if (currentStep == 4 && !follower.isBusy()) {
                actionTimer.reset(); // Clear the timer so we can count how long to stay there
                currentStep = 5;
            }

            // STEP 5: Stay at the intake location for 2 more seconds with intake spinning
            if (currentStep == 5) {
                intake.setPower(1.0); // Keeps intake running while stationary

                if (actionTimer.seconds() >= 2.0) {
                    intake.setPower(0.0); // Shut off intake motor completely
                    schedule(follow(follower, path3())); // Schedule driving to the parking spot
                    currentStep = 6;
                }
            }

            // STEP 6: Wait until the robot finishes parking, then do nothing else
            if (currentStep == 6 && !follower.isBusy()) {
                currentStep = 7; // Completes the path and holds completely still
            }
        }
    }

    // Path definitions using only clean Paths.line builders to prevent errors

    // Path 1: Drive straight forward off the wall to clear the boundary safely
    public Path path1() {
        return Paths.line(startAndScorePose, wallClearancePose).linear(startAndScorePose, wallClearancePose);
    }

    // Path 2: Drive left to the ground balls while rotating heading to 180 degrees
    public Path path2() {
        return Paths.line(wallClearancePose, intakePose).linear(wallClearancePose, intakePose);
    }

    // Path 3: Drive out of the intake zone to the park location
    public Path path3() {
        return Paths.line(intakePose, parkPose).linear(intakePose, parkPose);
    }
}
