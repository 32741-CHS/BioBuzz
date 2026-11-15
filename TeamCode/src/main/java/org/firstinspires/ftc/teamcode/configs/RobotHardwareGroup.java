package org.firstinspires.ftc.teamcode.configs;

import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;
import static java.util.Objects.isNull;

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
        flDrive = safeGet(DcMotor.class, FL_MOTOR_NAME);
        frDrive = safeGet(DcMotor.class, FR_MOTOR_NAME);
        blDrive = safeGet(DcMotor.class, BL_MOTOR_NAME);
        brDrive = safeGet(DcMotor.class, BR_MOTOR_NAME);
        turret = safeGet(DcMotor.class, TURRET_MOTOR_NAME);
        flywheel = safeGet(DcMotor.class, FLYWHEEL_MOTOR_NAME);
        feeder = safeGet(DcMotor.class, FEEDER_MOTOR_NAME);
        intake = safeGet(DcMotor.class, INTAKE_MOTOR_NAME);
        imu = safeGet(IMU.class, IMU_NAME);
        vision = safeGet(WebcamName.class, MAIN_WEBCAM_NAME);
        insideCam = safeGet(WebcamName.class, INSIDE_WEBCAM_NAME);
        pinpoint = safeGet(GoBildaPinpointDriver.class, PINPOINT_NAME);
        servo = safeGet(Servo.class, SERVO_NAME);
    }

    private <T> T safeGet(Class<? extends T> classOrInterface, String name) {
        T obj = hardwareMap.tryGet(classOrInterface, name);
        if (isNull(obj))
            panelsTelemetry.addData(
                    "[ERROR] " + name,
                    "FAILED TO FIND  HARDWARE (Ignoring)" +
                            " \n Try Checking if the wire is connected firmly, or the driver hub hardware config name matches: " + name
            );
        return obj;
    }
}