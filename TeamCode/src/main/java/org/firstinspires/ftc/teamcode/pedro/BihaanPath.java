package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

// Changed class name to match your file name
public class BihaanPath {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(12, 12, 90);
    private final Pose path1 = poseFactory.of(59.0559, 35.3219, 180);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }
}
