package org.firstinspires.ftc.teamcode.opmode.Auto.pedro;

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

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("LF");
        c.frontRightName.set("RF");
        c.backLeftName.set("LB");
        c.backRightName.set("RB");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(2.800179128571758);
        c.yPodOffset.set(0.9196552516907219);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2812461114961419);
                Controller secondaryTranslationalForward = Controller.proportional(0.1039129231633189);
                Controller primaryTranslationalLateral = Controller.proportional(0.36582945449642085);
                Controller secondaryTranslationalLateral = Controller.proportional(0.13516420829337197);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010692534678271566));
                c.brake.set(Controller.proportionalFeedforward(0.00908865447653083));

                c.headingFeedback.set(Controller.proportional(3.6009307445672363));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05875246931742394, 0.00353877872876127));

                c.linearBrakeCoefficients.set(Matrix.diag(0.1109373774320622, 0.0655697369372207));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0017255553989828731, 0.00206713804032446));

                c.maxAchievableForwardVelocity.set(83.97308964363148);
                c.maxAchievableStrafeVelocity.set(71.01353264996729);
                c.naturalForwardDeceleration.set(24.263385502984725);
                c.naturalStrafeDeceleration.set(53.12674841520208);
            }
    );

}