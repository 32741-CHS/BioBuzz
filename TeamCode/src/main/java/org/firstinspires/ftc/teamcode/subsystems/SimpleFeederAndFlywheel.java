package org.firstinspires.ftc.teamcode.subsystems;
// TODO Finish

import static org.firstinspires.ftc.teamcode.configs.Globals.GOBILDA_5203_6000RPM;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

@Configurable
public class SimpleFeederAndFlywheel implements SubsystemBase {

    private DcMotorEx feeder;
    private DcMotor intake;
    private DcMotorEx flywheel;

    @Override
    public void init(RobotHardwareGroup robotHardware) {
        feeder = (DcMotorEx) robotHardware.intake;
        feeder.setDirection(DcMotorSimple.Direction.FORWARD);
        feeder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setPower(1);

        flywheel = (DcMotorEx) robotHardware.intake;
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void update() {
        feeder.setPositionPIDFCoefficients((int) Math.round(GOBILDA_5203_6000RPM / 2.0));
    }
}

