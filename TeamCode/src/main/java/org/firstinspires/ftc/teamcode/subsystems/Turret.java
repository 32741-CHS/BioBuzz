package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

@Configurable
public class Turret implements SubsystemBase {

    public static final double desiredPower = 1;
    public static boolean isInverted = false;
    private static double targetPower;
    private DcMotor intake;

    @Override
    public void init(RobotHardwareGroup robotHardware) {

    }

    @Override
    public void update() {
        intake.setPower(!isInverted ? targetPower : -targetPower);
        targetPower = 0;
        isInverted = false;
    }
}
