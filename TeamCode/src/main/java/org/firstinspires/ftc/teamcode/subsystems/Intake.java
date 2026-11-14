package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.MotorBasicActions;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

@Configurable
public class Intake implements SubsystemBase, MotorBasicActions {

    private boolean reverseMode = false;
    private DcMotor intake;

    @Override
    public void init(RobotHardwareGroup robotHardware) {
        intake = robotHardware.intake;
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void update() {
    }

    public void tryRunMotorIfReady() {
        intake.setPower(reverseMode ? 1 : 0);
    }

    public void stopMotor() {
        intake.setPower(0);
    }

    public void reverseMode() {
        reverseMode = true;
    }

    public void deactivateReverseMode() {
        reverseMode = false;
    }

}
