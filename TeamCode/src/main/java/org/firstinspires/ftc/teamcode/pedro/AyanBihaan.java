package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

@Autonomous(name = "Teammate Paths: Ayan & Bihaan", group = "Autonomous")
public class AyanBihaan extends LinearOpMode {

    private Follower follower;
    private AyanPath ayanPath;   // Renamed from ayanContainer
    private BihaanPath bihaanPath; // Renamed from bihaanContainer
    private boolean runAyan = true;

    @Override
    public void runOpMode() {
        // Initialize the driving and tracking engine
        follower = Constants.create(hardwareMap);

        // Instantiate both teammate path coordinate files
        ayanPath = new AyanPath();
        bihaanPath = new BihaanPath();

        // Set the starting position to match the visualizer start position (12, 12, 90 degrees)
        Pose startPose = new Pose(12, 12, Math.toRadians(90));
        follower.setPose(startPose);

        // Selection screen loop (Runs AFTER you press Init, but BEFORE you press Play)
        while (!isStarted() && !isStopRequested()) {

            // If the user presses X, select Ayan's path
            if (gamepad1.x) {
                runAyan = true;
            }

            // If the user presses Y, select Bihaan's path
            if (gamepad1.y) {
                runAyan = false;
            }

            // Print the current selection directly onto the Driver Station screen
            if (runAyan == true) {
                telemetry.addData("Selected Path", "AYAN (Curve)");
            } else {
                telemetry.addData("Selected Path", "BIHAAN (Line)");
            }

            telemetry.update();
        }

        // Wait until the user physically taps the Play button on the controller
        waitForStart();
        if (isStopRequested()) return;

        // Execute the path depending on which teammate was chosen
        if (runAyan == true) {
            follower.follow(ayanPath.path1());
        } else {
            follower.follow(bihaanPath.path1());
        }

        // Active driving loop (Runs until the robot finishes the path or you hit Stop)
        while (opModeIsActive() && !isStopRequested()) {
            // Live hardware position updates via the Pinpoint odometry computer
            follower.update();
        }
    }
}
