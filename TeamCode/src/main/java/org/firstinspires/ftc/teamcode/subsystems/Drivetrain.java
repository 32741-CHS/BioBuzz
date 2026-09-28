package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.Globals.GOBILDA_5203_312RPM;
import static org.firstinspires.ftc.teamcode.configs.Globals.STICK_DEADBAND;
import static org.firstinspires.ftc.teamcode.configs.Globals.WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.configs.Globals.robotHardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** Handles movement of the 4 mecanum wheels */
public class Drivetrain {

    //    /** How many encoder ticks equal one centimeter of physical travel */
    //    static final double COUNTS_PER_CM = GOBILDA_5203_312RPM / (WHEEL_DIAMETER * Math.PI); //

    private static final double SPEED_SLOW = 0.4;
    private static final double SPEED_NORMAL = 1;
    public static double AUTO_DRIVE_SPEED = 0.4;
    private final DcMotor flDrive, frDrive, blDrive, brDrive;
    private final IMU imu;
    public double Expected_X = 0;
    public double Expected_Y = 0;
    /**
     * Tracks the elapsed time since the drive() function has been called gets set to 0 when using
     * Shift functions so beware
     */
    public double PrevLastMoved = 0;
    public double LastMoved = 0;
    private double speedMultiplier = SPEED_NORMAL;

    public Drivetrain() {
        (flDrive = robotHardware.flDrive).setDirection(Direction.REVERSE);
        (blDrive = robotHardware.blDrive).setDirection(Direction.REVERSE);
        (frDrive = robotHardware.frDrive).setDirection(Direction.FORWARD);
        (brDrive = robotHardware.brDrive).setDirection(Direction.FORWARD);
        imu = robotHardware.imu;

        for (DcMotor m : new DcMotor[] {flDrive, frDrive, blDrive, brDrive}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        resetIMU();
    }

    /**
     * Normalizes speed (meters per second) to a value from 0 to 1 makes it viable to use for
     * DcMotor.setPower()
     */
    private float NormalizeSpeed(float speed) {
        float WHEEL_RADIUS = (float) WHEEL_DIAMETER / 2;
        float power = (60 * speed) / (WHEEL_RADIUS * (float) Math.PI * (float) GOBILDA_5203_312RPM);
        return power > 1 ? 1 : power;
    }

    /** converts power to speed (meters poer second) */
    private float PowerToSpeed(float power) {
        float WHEEL_RADIUS = (float) WHEEL_DIAMETER / 2;
        float speed = (float) Math.PI * WHEEL_RADIUS / 60 * (float) GOBILDA_5203_312RPM * power;
        return speed;
    }

    /**
     * takes in joystick input and applies to all motors thanks omar and kiaan
     *
     * @param y The Joystick Y
     * @param x The Joystick X
     * @param rx
     * @param fieldRelative
     */
    public void drive(double y, double x, double rx, boolean fieldRelative) {
        PrevLastMoved = LastMoved;
        LastMoved = System.currentTimeMillis();

        y = deadband(y);
        x = deadband(x) * 1.1;
        rx = deadband(rx);

        if (fieldRelative) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);
            double rotX = x * cos - y * sin;
            double rotY = x * sin + y * cos;
            x = rotX;
            y = rotY;
        }

        double fl = (y + x + rx);
        double fr = (y - x - rx);
        double bl = (y - x + rx);
        double br = (y + x - rx);

        double max =
                Math.max(
                        1.0,
                        Math.max(
                                Math.abs(fl),
                                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        double flSpeed = (fl / max) * speedMultiplier;
        flDrive.setPower(flSpeed);

        double frSpeed = (fr / max) * speedMultiplier;
        frDrive.setPower(frSpeed);

        double blSpeed = (bl / max) * speedMultiplier;
        blDrive.setPower(blSpeed);

        double brSpeed = (br / max) * speedMultiplier;
        brDrive.setPower(brSpeed);

        // TODO find a way to update Expected_X and Expected_Y based on the movement of the drive
    }

    private double deadband(double value) {
        return Math.abs(value) < STICK_DEADBAND ? 0 : value;
    }

    /** shifts the robot forward in reverse based off direction default is forward */
    public void Shift(float speed, Direction dir) {
        PrevLastMoved = LastMoved;
        LastMoved = System.currentTimeMillis();
        flDrive.setDirection(dir);
        frDrive.setDirection(dir);

        if (dir == Direction.FORWARD) {
            Expected_Y += speed;
        } else {
            Expected_Y -= speed;
        }

        float power = NormalizeSpeed(speed);
        flDrive.setPower(power);
        frDrive.setPower(power);
    }

    public void Shift(float speed) {
        Shift(speed, Direction.FORWARD);
    }

    /** updates the movement of the robot to be to their right */
    public void ShiftRight(float speed) {
        PrevLastMoved = LastMoved;
        LastMoved = System.currentTimeMillis();
        frDrive.setDirection(Direction.REVERSE);
        brDrive.setDirection(Direction.FORWARD);

        Expected_X += speed;

        float power = NormalizeSpeed(speed);
        frDrive.setPower(power);
        brDrive.setPower(power);
    }

    /**
     * updates the movement of the robot to be their left
     *
     * @param speed in ...
     */
    public void ShiftLeft(float speed) {
        PrevLastMoved = LastMoved;
        LastMoved = System.currentTimeMillis();
        flDrive.setDirection(Direction.REVERSE);
        blDrive.setDirection(Direction.FORWARD);

        Expected_X -= speed;

        float power = NormalizeSpeed(speed);
        flDrive.setPower(power);
        blDrive.setPower(power);
    }

    public void resetIMU() {
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.DOWN)));
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    public void setSpeedMultiplier(double newSpeed) {
        speedMultiplier = newSpeed;
    }

    public void stop() {
        speedMultiplier = 0;
    }
}
