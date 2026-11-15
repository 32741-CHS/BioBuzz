package org.firstinspires.ftc.teamcode.configs;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

// all hardware goes HERE!!! if wiring changes, only this file needs updating.
public class RobotHardwareGroup {
    public final static String
            FL_MOTOR_NAME = "flDrive",
            FR_MOTOR_NAME = "frDrive",
            BL_MOTOR_NAME = "blDrive",
            BR_MOTOR_NAME = "brDrive",
            TURRET_MOTOR_NAME = "turret",
            FLYWHEEL_MOTOR_NAME = "flywheel",
            FEEDER_MOTOR_NAME = "feeder",
            INTAKE_MOTOR_NAME = "intake",
            IMU_NAME = "imu",
            PINPOINT_NAME = "pinpoint",
            SERVO_NAME = "servo",
            MAIN_WEBCAM_NAME = "vision",
            INSIDE_WEBCAM_NAME = "insideCam";

    public final HardwareMap hardwareMap;
    public final @Nullable DcMotor flDrive, frDrive, blDrive, brDrive, feeder, intake, turret, flywheel;
    public final @Nullable IMU imu;
    public final @Nullable WebcamName vision, insideCam;
    public final @Nullable GoBildaPinpointDriver pinpoint;
    public final @Nullable Servo servo;

    public RobotHardwareGroup(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        flDrive = hardwareMap.tryGet(DcMotor.class, FL_MOTOR_NAME);
        frDrive = hardwareMap.tryGet(DcMotor.class, FR_MOTOR_NAME);
        blDrive = hardwareMap.tryGet(DcMotor.class, BL_MOTOR_NAME);
        brDrive = hardwareMap.tryGet(DcMotor.class, BR_MOTOR_NAME);
        turret = hardwareMap.tryGet(DcMotor.class, TURRET_MOTOR_NAME);
        flywheel = hardwareMap.tryGet(DcMotor.class, FLYWHEEL_MOTOR_NAME);
        feeder = hardwareMap.tryGet(DcMotor.class, FEEDER_MOTOR_NAME);
        intake = hardwareMap.tryGet(DcMotor.class, INTAKE_MOTOR_NAME);
        imu = hardwareMap.tryGet(IMU.class, IMU_NAME);
        vision = hardwareMap.tryGet(WebcamName.class, MAIN_WEBCAM_NAME);
        insideCam = hardwareMap.tryGet(WebcamName.class, INSIDE_WEBCAM_NAME);
        pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, PINPOINT_NAME);
        servo = hardwareMap.tryGet(Servo.class, SERVO_NAME);
    }
}