package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

@Configurable
public class Intake implements SubsystemBase {

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

    public void run() {
        intake.setPower(reverseMode ? 1 : 0);
    }

    public void stopRunning() {
        intake.setPower(0);
    }

    public void reverse() {
        reverseMode = true;
    }

    public void unReverse() {
        reverseMode = false;
    }

}
