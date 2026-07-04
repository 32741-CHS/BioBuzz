package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.TickRates.GOBILDA_5203_312RPM;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.configs.RobotHardware;

public class Drivetrain {

    static final double WHEEL_DIAMETER = 10.4;
    static final double COUNTS_PER_CM = GOBILDA_5203_312RPM / (WHEEL_DIAMETER * Math.PI);

    private static final double SPEED_SLOW   = 0.4;
    private static final double SPEED_NORMAL = 1;

    private static final double STICK_DEADBAND = 0.05;

    public static double AUTO_DRIVE_SPEED = 0.4;

    private final DcMotor flDrive, frDrive, blDrive, brDrive;
    private final IMU imu;

    private double speedMultiplier = SPEED_NORMAL;


    public Drivetrain(RobotHardware hw) {
        (flDrive = hw.flDrive).setDirection(DcMotorSimple.Direction.REVERSE);
        (blDrive = hw.blDrive).setDirection(DcMotorSimple.Direction.REVERSE);
        (frDrive = hw.frDrive).setDirection(DcMotorSimple.Direction.FORWARD);
        (brDrive = hw.brDrive).setDirection(DcMotorSimple.Direction.FORWARD);
        imu = hw.imu;

        for (DcMotor m : new DcMotor[]{flDrive, frDrive, blDrive, brDrive}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        resetIMU();
    }

    public void drive(double y, double x, double rx, boolean fieldRelative) {
        y  = deadband(y);
        x  = deadband(x) * 1.1;
        rx = deadband(rx);

        if (fieldRelative) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);
            double rotX = x * cos - y * sin;
            double rotY = x * sin + y * cos;
            x  = rotX;
            y  = rotY;
        }

        double fl = (y + x + rx);
        double fr = (y - x - rx);
        double bl = (y - x + rx);
        double br = (y + x - rx);

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                         Math.max(Math.abs(fr),
                         Math.max(Math.abs(bl), Math.abs(br)))));

        flDrive.setPower((fl / max) * speedMultiplier);
        frDrive.setPower((fr / max) * speedMultiplier);
        blDrive.setPower((bl / max) * speedMultiplier);
        brDrive.setPower((br / max) * speedMultiplier);
    }

    public void setSpeedMultiplier(boolean slow) {
        speedMultiplier = (slow ? SPEED_SLOW : SPEED_NORMAL);
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    public void resetIMU() {
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                )
        ));
    }

    private double deadband(double v) {
        return Math.abs(v) < STICK_DEADBAND ? 0 : v;
    }

    public void stop() {
        for (DcMotor m : new DcMotor[]{flDrive, frDrive, blDrive, brDrive}) {
            m.setPower(0);
        }
    }

    // async + non blocking methods (iterative opmodes, state machines (need for pedro later))
    public void setDriveTarget(double distance) {
        int target = (int)(distance * COUNTS_PER_CM);

        for (DcMotor m : new DcMotor[]{flDrive, frDrive, blDrive, brDrive}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setTargetPosition(target);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(AUTO_DRIVE_SPEED);
        }
    }

    public boolean isBusy() {
        // Returns true if ANY motor is still driving to its target
        return flDrive.isBusy() || frDrive.isBusy() || blDrive.isBusy() || brDrive.isBusy();
    }

    public void clearDriveTarget() {
        for (DcMotor m : new DcMotor[]{flDrive, frDrive, blDrive, brDrive}) {
            m.setPower(0);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // synchronous / blocking method (best for linearopmodes)
    public void driveDistanceBlocking(double distance, LinearOpMode opMode) {
        setDriveTarget(distance);
        
        while (opMode.opModeIsActive() && isBusy()) {
            // Yield to other processes while waiting
            opMode.idle(); 
        }
        
        clearDriveTarget();
    }
}
