package org.firstinspires.ftc.teamcode.configs;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.NoSuchElementException;

// all hardware goes HERE!!! if wiring changes, only this file needs updating.

public class BotHardware {
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
    public final @Nullable DcMotor
            flDrive,
            frDrive,
            blDrive,
            brDrive,
            feeder,
            intake,
    //            turret,
    flywheel;
    public final @Nullable IMU imu;
    public final @Nullable WebcamName vision, insideCam;
    public final @Nullable GoBildaPinpointDriver pinpoint;
    public final @Nullable Servo servo;

    public BotHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        flDrive = safeGet(DcMotor.class, FL_MOTOR_NAME);
        frDrive = safeGet(DcMotor.class, FR_MOTOR_NAME);
        blDrive = safeGet(DcMotor.class, BL_MOTOR_NAME);
        brDrive = safeGet(DcMotor.class, BR_MOTOR_NAME);
//        turret = safeGet(DcMotor.class, TURRET_MOTOR_NAME);
        flywheel = safeGet(DcMotor.class, FLYWHEEL_MOTOR_NAME);
        feeder = safeGet(DcMotor.class, FEEDER_MOTOR_NAME);
        intake = safeGet(DcMotor.class, INTAKE_MOTOR_NAME);
        imu = safeGet(IMU.class, IMU_NAME);
        vision = safeGet(WebcamName.class, MAIN_WEBCAM_NAME);
        insideCam = safeGet(WebcamName.class, INSIDE_WEBCAM_NAME);
        pinpoint = safeGet(GoBildaPinpointDriver.class, PINPOINT_NAME);
        servo = safeGet(Servo.class, SERVO_NAME);

        /// ToDo: Too lazy to do this properly
        DcMotor.RunMode
                noEn = DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                yesEn = DcMotor.RunMode.RUN_USING_ENCODER,
                yesPosEn = DcMotor.RunMode.RUN_TO_POSITION;

        DcMotorSimple.Direction
                dirF = DcMotorSimple.Direction.FORWARD,
                dirR = DcMotorSimple.Direction.REVERSE;

        DcMotor.ZeroPowerBehavior
                brake = DcMotor.ZeroPowerBehavior.BRAKE,
                roll = DcMotor.ZeroPowerBehavior.FLOAT;

        if (flDrive != null) configMotor(flDrive, dirR, brake, noEn);
        if (frDrive != null) configMotor(frDrive, dirF, brake, noEn);
        if (blDrive != null) configMotor(blDrive, dirR, brake, noEn);
        if (brDrive != null) configMotor(brDrive, dirF, brake, noEn);
//        if (turret != null) configMotor(turret, dirF, brake, noEn);
        if (flywheel != null) configMotor(flywheel, dirF, roll, noEn);
        if (feeder != null) configMotor(feeder, dirF, brake, yesPosEn);
        if (intake != null) configMotor(intake, dirF, brake, noEn);
    }

    private @Nullable <T> T safeGet(Class<? extends T> classOrInterface, String name) {
        T obj = hardwareMap.get(classOrInterface, name);
        if (obj == null)
            throw new NoSuchElementException("There is no hardware with the name " + name);
        return obj;
    }

    /// Normal Motor Config
    private void configMotor(DcMotor motor, DcMotorSimple.Direction direction, DcMotor.ZeroPowerBehavior zeroPowerBehavior, DcMotor.RunMode runMode) {
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.setMode(runMode);
    }

    /// PIDF Motor Config
    private void configMotor(DcMotor motor, DcMotorSimple.Direction direction, DcMotor.ZeroPowerBehavior zeroPowerBehavior, boolean isVelPIDF, PIDFCoefficients coefficients) {
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        ((DcMotorEx) motor).setPIDFCoefficients(
                isVelPIDF ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_TO_POSITION,
                coefficients
        );
    }
}