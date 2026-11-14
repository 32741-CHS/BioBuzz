package org.firstinspires.ftc.teamcode.subsystems;
// TODO Finish

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

@Configurable
public class SimpleFeederIntakeAndFlywheel implements SubsystemBase {

    private DcMotorEx feeder;
    private DcMotor intake;
    private DcMotorEx flywheel;

    @Override
    public void init(RobotHardwareGroup robotHardware) {
        feeder = (DcMotorEx) robotHardware.intake;
        intake = robotHardware.intake;
        flywheel = (DcMotorEx) robotHardware.intake;
    }

    @Override
    public void update() {
    }
}

class MotorExHelper {
    DcMotorEx motor;


    public MotorExHelper(DcMotorEx motor, Boolean isReverse, Boolean breakOnStop) {
        motor.setDirection(isReverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(breakOnStop ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor = motor;
    }


}
