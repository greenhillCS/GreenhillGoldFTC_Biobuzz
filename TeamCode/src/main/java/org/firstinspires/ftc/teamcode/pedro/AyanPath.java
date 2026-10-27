package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

// Changed class name to match your file name
public class AyanPath {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(12, 12, 90);
    private final Pose path1 = poseFactory.of(41.877, 112.1749, 180);
    private final Pose path1Control1 = poseFactory.of(22.3075, 75.6294, 0);

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }
}
