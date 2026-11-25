package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

// all hardware goes HERE!!! if wiring changes, only this file needs updating.
public enum BetterBotHardware {
    FL_DRIVE("flDrive"),
    FR_DRIVE("frDrive"),
    BL_DRIVE("blDrive"),
    BR_DRIVE("brDrive"),
    TURRET_MOTOR("turret"),
    FLYWHEEL_MOTOR("flywheel"),
    FEEDER_MOTOR("feeder"),
    INTAKE_MOTOR("intake"),
    IMU("imu"),
    PINPOINT("pinpoint"),
    SERVO("servo"),
    MAIN_WEBCAM("vision"),
    INSIDE_WEBCAM("insideCam");

    private static HardwareMap ftcHardwareMap;
    private final String name;
    private PIDFCoefficients coefficients;
    private boolean isVelocityPidfMode = false;

    BetterBotHardware(String name) {
        this.name = name;
    }

    BetterBotHardware(String name, boolean isVelocityPidfMode, PIDFCoefficients coefficients) {
        this.name = name;
        this.isVelocityPidfMode = isVelocityPidfMode;
        this.coefficients = coefficients;
    }

    public <T> T get(Class<? extends T> classOrInstance) {
        T obj = ftcHardwareMap.get(classOrInstance, name);
        if (obj instanceof DcMotor & coefficients != null)
            if (isVelocityPidfMode)
                ((DcMotorEx) obj).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
            else
                ((DcMotorEx) obj).setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, coefficients);
        return obj;
    }

    public String getName() {
        return name;
    }

    public void init(HardwareMap ftcHardwareMap) {
        BetterBotHardware.ftcHardwareMap = ftcHardwareMap;
    }
}