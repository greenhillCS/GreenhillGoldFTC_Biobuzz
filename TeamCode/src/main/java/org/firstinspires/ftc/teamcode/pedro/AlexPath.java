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

@Autonomous(name = "alexPath", group = "Autonomous")
public class AlexPath extends LinearOpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(17.1613, 66.6621, 180);
    private final Pose point2 = poseFactory.of(53.7697, 114.3973, 168.3748);
    private final Pose point2Control1 = poseFactory.of(28.3571, 119.9045, 0);
    private final Pose point3 = poseFactory.of(110.8636, 98.1356, 164.1018);
    private final Pose point4 = poseFactory.of(107.2616, 33.1453, 66.2016);
    private final Pose point4Control1 = poseFactory.of(118.6653, 58.7159, 0);
    private final Pose point5 = poseFactory.of(77.9117, 18.4294, 26.6289);
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
        return Paths.line(start, path1).linear(start, path1);
    }
    public Path path2() {
        return Paths.curve(path1, point2Control1, point2).reverseTangent();
    }
    public Path path3() {
        return Paths.line(point2, point3).reverseTangent();
    }
    public Path path4() {
        return Paths.curve(point3, point4Control1, point4).reverseTangent();
    }
    public Path path5() {
        return Paths.line(point4, point5).reverseTangent();
    }
}
