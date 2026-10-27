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

@Autonomous(name = "ayanPath", group = "Autonomous")
public class AyanPath extends LinearOpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(23.748, 50.0819, 180);
    private final Pose path1Control1 = poseFactory.of(23.5024, 22.2905, 0);
    private final Pose point2 = poseFactory.of(21.5177, 117.3499, -88.101);
    private final Pose point3 = poseFactory.of(115.8604, 115.8515, 179.0901);
    private final Pose point4 = poseFactory.of(112.2584, 47, 87.0053);
    private final Pose point5 = poseFactory.of(87.9053, 23.6533, 9.3094);
    private final Pose point5Control1 = poseFactory.of(108.3058, 26.8997, 0);
    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5())
        );
    }
    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        waitForStart();
        schedule(autoRoutine());
        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }
            telemetry.update();
        }
    }
    public Path path1() {
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }
    public Path path2() {
        return Paths.line(path1, point2).reverseTangent();
    }
    public Path path3() {
        return Paths.line(point2, point3).reverseTangent();
    }
    public Path path4() {
        return Paths.line(point3, point4).reverseTangent();
    }
    public Path path5() {
        return Paths.curve(point4, point5Control1, point5).reverseTangent();
    }
}
