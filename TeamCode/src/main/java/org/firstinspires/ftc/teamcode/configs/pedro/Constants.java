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
        c.xPodOffset.set(-1.31825255373095);
        c.yPodOffset.set(-5.9822952451323275);
        c.forwardTicksToInches.set(1.7682112675608357);
        c.strafeTicksToInches.set(0.002141540966247379);
        c.xPodDirection.set(Encoder.REVERSE);
        c.yPodDirection.set(Encoder.FORWARD);
        c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
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