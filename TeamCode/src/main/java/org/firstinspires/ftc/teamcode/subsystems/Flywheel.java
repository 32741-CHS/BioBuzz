package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.Globals.FLYWHEEL_ERROR_TOL;
import static org.firstinspires.ftc.teamcode.configs.Globals.GOBILDA_5203_6000RPM;
import static org.firstinspires.ftc.teamcode.configs.Globals.panelsGraph;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.configs.Globals;
import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

@Configurable
public class Flywheel implements SubsystemBase {
    public static double desiredFlywheelRPS = 13;
    public static boolean canSpinFlywheel = false;
    private DcMotorEx flywheel;


    public Flywheel() {
    }

    public static void setDesiredFlywheelRPS(double rps) {
        desiredFlywheelRPS = rps;
    }

    @Override
    public void init(RobotHardwareGroup robotHardware) {
        flywheel = robotHardware.flywheel;
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void update() {
        // apply PIDF every frame so @Configurable changes work
        flywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(Globals.flywheelKP, 0, 0, Globals.flywheelKF));

        flywheel.setVelocity(canSpinFlywheel ? desiredFlywheelRPS * GOBILDA_5203_6000RPM : 0);

        panelsGraph.addData("Flywheel RPS", getFlywheelRPS());
        panelsGraph.addData("Flywheel Error", getFlywheelErrorRPS());
    }

    public double getFlywheelRPS() {
        return flywheel.getVelocity() / GOBILDA_5203_6000RPM;
    }

    public double getFlywheelErrorRPS() {
        return getFlywheelRPS() - desiredFlywheelRPS;
    }

    public boolean isReadyForFeed() {
        return Math.abs(getFlywheelErrorRPS()) <= FLYWHEEL_ERROR_TOL;
    }


}
