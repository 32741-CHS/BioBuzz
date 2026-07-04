package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.TickRates.GOBILDA_5203_312RPM;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.configs.RobotHardware;

@Configurable
public class Turret {

    private final DcMotorEx turret;
    // TODO: measure wheel diameter / lazy susan diameter
    private final static double MOTOR_TO_TURRET_RATIO = 65.5 / 290;

    public static double desiredAngle;
    public static double kP = 0.03;
    public static double kF = 0.04;
    public static double MAX_POWER = 0.6;
    public static double MAX_ACCEL = 0.02;
    public static double ANGLE_TOLERANCE = 1.5;
    public static double MIN_ANGLE = -80;
    public static double MAX_ANGLE = 80;
    public static double SOFT_ZONE_DEG = 5.0;

    private double lastPower = 0;

    public Turret(RobotHardware hw) {
        turret = hw.turret;
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getCurrentAngle() {
        int ticks = turret.getCurrentPosition();
        return ticks / GOBILDA_5203_312RPM * MOTOR_TO_TURRET_RATIO * 360;
    }

    public double getErrorAngle() {
        return desiredAngle - getCurrentAngle();
    }

    public void goTo(double angle) {
        desiredAngle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
    }

    public void resetTurretEncoder() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        desiredAngle = 0;
    }

    public void update() {
        double currentAngle = getCurrentAngle();
        double error = getErrorAngle();

        if (Math.abs(error) < ANGLE_TOLERANCE) {
            turret.setPower(0);
            lastPower = 0;
            return;
        }

        double p = kP * error;

        double ff = Math.copySign(kF, error);

        double rawPower = p + ff;

        double distToMin = currentAngle - MIN_ANGLE;
        double distToMax = MAX_ANGLE - currentAngle;
        double speedLimit = MAX_POWER;

        if (error < 0 && distToMin < SOFT_ZONE_DEG) {
            // moving toward minimum limit
            speedLimit *= Math.max(0.1, distToMin / SOFT_ZONE_DEG);
        } else if (error > 0 && distToMax < SOFT_ZONE_DEG) {
            // moving toward maximum limit
            speedLimit *= Math.max(0.1, distToMax / SOFT_ZONE_DEG);
        }

        double clipped = Range.clip(rawPower, -speedLimit, speedLimit);

        double delta = clipped - lastPower;
        delta = Range.clip(delta, -MAX_ACCEL, MAX_ACCEL);
        double power = lastPower + delta;

        turret.setPower(power);
        lastPower = power;
    }
}
