package org.firstinspires.ftc.teamcode.configs.pedro;

/// Generated code goes here!! Go to Tuning to see how to generate code

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static TwoWheelConfig localizerConfig = new TwoWheelConfig(c -> {
        c.xPodName.set("frDrive");
        c.yPodName.set("flDrive");
        c.imuName.set("imu");
        c.xPodOffset.set(1.699039449722557);
        c.yPodOffset.set(11.452578902524735);
        c.forwardTicksToInches.set(23.02115724178473);
        c.strafeTicksToInches.set(0.0021213702775474907);
        c.xPodDirection.set(Encoder.FORWARD);
        c.yPodDirection.set(Encoder.REVERSE);
        c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )));
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

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new TwoWheelLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                null
        );
    }
}