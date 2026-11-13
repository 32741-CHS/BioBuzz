package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    /// Archaic for backwards compatibility
    TURRET_MOTOR_NAME = "turret",
            FLYWHEEL_MOTOR_NAME = "flywheel",
            FEEDER_MOTOR_NAME = "feeder",
            INTAKE_MOTOR_NAME = "intake",
            IMU_NAME = "imu",
            PINPOINT_NAME = "pinpoint",
            SERVO_NAME = "servo",
            MAIN_WEBCAM_NAME = "webcam";

    public HardwareMap hardwareMap;
    public DcMotor flDrive, frDrive, blDrive, brDrive, feeder, intake;
    public DcMotorEx turret, flywheel;
    public IMU imu;
    public WebcamName vision, insideCam;
    public GoBildaPinpointDriver pinpoint;
    public Servo servo;

    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        flDrive = hardwareMap.get(DcMotor.class, FL_MOTOR_NAME);
        frDrive = hardwareMap.get(DcMotor.class, FR_MOTOR_NAME);
        blDrive = hardwareMap.get(DcMotor.class, BL_MOTOR_NAME);
        brDrive = hardwareMap.get(DcMotor.class, BR_MOTOR_NAME);
//        turret = hardwareMap.get(DcMotorEx.class, TURRET_MOTOR_NAME);
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR_NAME);
        feeder = hardwareMap.get(DcMotor.class, FEEDER_MOTOR_NAME);
        intake = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        imu = hardwareMap.get(IMU.class, IMU_NAME);
//        vision = hardwareMap.get(WebcamName.class, MAIN_WEBCAM_NAME);
//        insideCam = hardwareMap.get(WebcamName.class, INSIDE_WEBCAM_NAME);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        servo = hardwareMap.get(Servo.class, SERVO_NAME);
    }
}
