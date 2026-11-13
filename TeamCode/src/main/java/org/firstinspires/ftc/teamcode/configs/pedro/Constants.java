package org.firstinspires.ftc.teamcode.configs.pedro;

/// Generated code goes here!! Go to Tuning to see how to generate code

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

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(2.6310820091427782);
        c.yPodOffset.set(-5.192090732844796);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("flDrive");
        c.frontRightName.set("frDrive");
        c.backLeftName.set("blDrive");
        c.backRightName.set("brDrive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.40321910510257797);
                Controller secondaryTranslationalForward = Controller.proportional(0.14897868512248272);
                Controller primaryTranslationalLateral = Controller.proportional(0.6281726244225341);
                Controller secondaryTranslationalLateral = Controller.proportional(0.23209299964246652);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.018221060737500665));
                c.brake.set(Controller.proportionalFeedforward(0.015487901626875565));

                c.headingFeedback.set(Controller.proportional(3.792046674747035));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05008521764706758, 0.013932760431915651));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11122314330160046, 0.0914507055088411));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0020047865678331557, 0.0026963471883043066));

                c.maxAchievableForwardVelocity.set(38.5361698979939);
                c.maxAchievableStrafeVelocity.set(30.13904633009808);
                c.naturalForwardDeceleration.set(28.87333547811114);
                c.naturalStrafeDeceleration.set(20.027545536295825);
            }
    );

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}