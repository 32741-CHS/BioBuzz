package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.Globals.FEEDER_POWER;
import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.MotorBasicActions;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

@Configurable
public class Feeder implements SubsystemBase, MotorBasicActions {
    private final Flywheel flywheel;
    private double desiredPower = 0;
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
        feeder.setPower(desiredPower);
        panelsTelemetry.addData("Feeder Power", reverseMode ? -FEEDER_POWER : FEEDER_POWER);
        panelsTelemetry.addData("Flywheel ready to feed?", flywheel.isReadyForFeed());
    }

    public void tryRunMotorIfReady() {
        if (flywheel.isReadyForFeed()) desiredPower = reverseMode ? -FEEDER_POWER : FEEDER_POWER;
    }

    public void stopMotor() {
        desiredPower = 0;
    }

    public void reverseMode() {
        reverseMode = true;
    }

    public void deactivateReverseMode() {
        reverseMode = false;
    }

}
