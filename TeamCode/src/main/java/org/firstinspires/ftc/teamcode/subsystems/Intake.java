package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.Globals.robotHardwareGroup;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
public class Intake {

    public static final double desiredPower = 1;
    public static boolean isInverted = false;
    private static double targetPower;
    private final DcMotor intake;

    public Intake() {
        intake = robotHardwareGroup.intake;
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
