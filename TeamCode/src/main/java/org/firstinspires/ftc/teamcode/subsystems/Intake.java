package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

@Configurable
public class Intake implements SubsystemBase {

    public static final double desiredPower = 1;
    public static boolean isInverted = false;
    private static double targetPower;
    private DcMotor intake;

    public void init(RobotHardwareGroup robotHardware) {
        intake = robotHardware.intake;
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
