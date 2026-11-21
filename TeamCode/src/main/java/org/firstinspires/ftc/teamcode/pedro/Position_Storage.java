package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;

public class Position_Storage {

    // Last robot position from PathARed, in the same map as TeleOp:
    // (0, 0, 0) = the shot 1 starting spot.
    public static Pose pos = new Pose(0, 0, 0);

    // True right after PathARed runs. TeleOp uses pos one time,
    // then sets this back to false so the next TeleOp starts fresh.
    public static boolean fromAuto = false;
}