package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.Globals.FEEDER_POWER;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

@Configurable
public class Feeder implements SubsystemBase {

    private final Flywheel flywheel;
    private boolean reverseMode = false;
    private DcMotor feeder;

    public Feeder(Flywheel flywheel) {
        this.flywheel = flywheel;

    }


    @Override
    public void init(RobotHardwareGroup robotHardware) {
        feeder = robotHardware.feeder;
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void update() {
    }

    public void tryRun() {
        if (flywheel.isReadyForFeed()) feeder.setPower(reverseMode ? -FEEDER_POWER : FEEDER_POWER);
    }

    public void stopRunning() {
        feeder.setPower(0);
    }

    public void reverse() {
        reverseMode = true;
    }

    public void unReverse() {
        reverseMode = false;
    }

}
