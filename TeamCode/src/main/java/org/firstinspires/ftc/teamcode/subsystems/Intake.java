package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.configs.RobotHardware;

@Configurable
public class Intake {

    private final DcMotor intake;

    private static double targetPower;
    public static double desiredPower = 1;

    public static boolean isInverted = false;

    public Intake(RobotHardware hw) {
        intake = hw.intake;
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void eat() {
        targetPower = desiredPower;
    }

    public double getPower() {
        return intake.getPower();
    }

    public void invert() {
        isInverted = true;
    }

    public void update() {
        intake.setPower(!isInverted ? targetPower : -targetPower);
        targetPower = 0;
        isInverted = false;
    }
}
