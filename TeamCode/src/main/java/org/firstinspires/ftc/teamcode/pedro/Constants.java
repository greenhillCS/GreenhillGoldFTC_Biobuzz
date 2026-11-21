
package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {

        c.frontLeftName.set("back_right_drive");
        c.frontRightName.set("back_left_drive");
        c.backLeftName.set("front_right_drive");
        c.backRightName.set("front_left_drive");

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        c.xPodOffset.set(-6.3831227219949564);
        c.yPodOffset.set(-0.24134859325378902);

        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);

        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward =
                Controller.proportional(0.23737987674420702);

        Controller secondaryTranslationalForward =
                Controller.proportional(0.0877055215498589);

        Controller primaryTranslationalLateral =
                Controller.proportional(0.3429402539351885);

        Controller secondaryTranslationalLateral =
                Controller.proportional(0.12670726029669982);

        c.forwardTranslational.set(
                Controller.piecewise(secondaryTranslationalForward)
                        .put(2.5, primaryTranslationalForward)
        );

        c.strafeTranslational.set(
                Controller.piecewise(secondaryTranslationalLateral)
                        .put(2.5, primaryTranslationalLateral)
        );

        c.coast.set(
                Controller.proportionalFeedforward(0.012102689138058557)
        );

        c.brake.set(
                Controller.proportionalFeedforward(0.010287285767349774)
        );

        c.headingFeedback.set(
                Controller.proportional(4.368273999096066)
        );

        c.headingBrakeCoefficients.set(
                Vector2D.cartesian(
                        0.057866911089694134,
                        0.00534895958023187
                )
        );

        c.linearBrakeCoefficients.set(
                Matrix.diag(
                        0.08314903316739683,
                        0.04272276812415275
                )
        );

        c.quadraticBrakeCoefficients.set(
                Matrix.diag(
                        0.0017064253591284088,
                        0.0022182829977645756
                )
        );

        c.maxAchievableForwardVelocity.set(79.7756255576534);
        c.maxAchievableStrafeVelocity.set(68.2282099778125);

        c.naturalForwardDeceleration.set(26.62734542780125);
        c.naturalStrafeDeceleration.set(58.45988386544958);
    });

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
