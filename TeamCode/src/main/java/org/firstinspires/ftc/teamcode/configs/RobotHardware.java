package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

// all hardware goes HERE!!! if wiring changes, only this file needs updating.
public class RobotHardware {

    public static DcMotor flDrive, frDrive, blDrive, brDrive, feeder, intake;
    public static DcMotorEx turret, flywheel;
    public static IMU imu;
    public static WebcamName vision;

    public void init(HardwareMap hardwareMap) {
        flDrive = hardwareMap.get(DcMotor.class, "flDrive");
        frDrive = hardwareMap.get(DcMotor.class, "frDrive");
        blDrive = hardwareMap.get(DcMotor.class, "blDrive");
        brDrive = hardwareMap.get(DcMotor.class, "brDrive");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel"); // Shooter
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        intake = hardwareMap.get(DcMotor.class, "intake");
        imu = hardwareMap.get(IMU.class, "imu");
        vision = hardwareMap.get(WebcamName.class, "webcam");
    }
}
