package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

// all hardware goes HERE!!! if wiring changes, only this file needs updating.
public class RobotHardwareGroup {

    public DcMotor flDrive, frDrive, blDrive, brDrive, feeder, intake;
    public DcMotorEx turret, flywheel;
    public IMU imu;
    public WebcamName vision;


    public final static String FL_MOTOR_NAME = "flDrive";
    public final static String FR_MOTOR_NAME = "frDrive";
    public final static String BL_MOTOR_NAME = "blDrive";
    public final static String BR_MOTOR_NAME = "brDrive";
    public final static String TURRET_MOTOR_NAME = "turret";
    public final static String FLYWHEEL_MOTOR_NAME = "flywheel";
    public final static String FEEDER_MOTOR_NAME = "feeder";
    public final static String INTAKE_MOTOR_NAME = "intake";
    public final static String IMU_NAME = "imu";
    public final static String MAIN_WEBCAM_NAME = "webcam";


    public void init(HardwareMap hardwareMap) {


        flDrive = hardwareMap.get(DcMotor.class, FL_MOTOR_NAME);
        frDrive = hardwareMap.get(DcMotor.class, FR_MOTOR_NAME);
        blDrive = hardwareMap.get(DcMotor.class, BL_MOTOR_NAME);
        brDrive = hardwareMap.get(DcMotor.class, BR_MOTOR_NAME);
        turret = hardwareMap.get(DcMotorEx.class, TURRET_MOTOR_NAME);
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR_NAME);
        feeder = hardwareMap.get(DcMotor.class, FEEDER_MOTOR_NAME);
        intake = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        imu = hardwareMap.get(IMU.class, IMU_NAME);
        vision = hardwareMap.get(WebcamName.class, MAIN_WEBCAM_NAME);
    }
}
