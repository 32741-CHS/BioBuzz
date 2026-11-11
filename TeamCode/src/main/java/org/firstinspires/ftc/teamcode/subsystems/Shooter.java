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
public class Shooter implements SubsystemBase {
    public static final double desiredFeederPower = 0.7;
    public static double desiredFlywheelRPS = 13;
    public static boolean canSpinFlywheel = false;
    private DcMotorEx flywheel;
    private DcMotor feeder;
    private boolean requestedFeed = false, requestedReverseFeed = false;

    public Shooter() {
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

        feeder = robotHardware.feeder;
        feeder.setDirection(DcMotorSimple.Direction.FORWARD);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void update() {
        // apply PIDF every frame so @Configurable changes work
        flywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(Globals.flywheelKP, 0, 0, Globals.flywheelKF));

        flywheel.setVelocity(canSpinFlywheel ? desiredFlywheelRPS * GOBILDA_5203_6000RPM : 0);

        if (requestedFeed && Math.abs(getFlywheelErrorRPS()) <= FLYWHEEL_ERROR_TOL) {
            feeder.setPower(requestedReverseFeed ? -desiredFeederPower : desiredFeederPower);
        } else {
            feeder.setPower(0);
        }
        requestedFeed = false;
        requestedReverseFeed = false;

        panelsGraph.addData("Flywheel RPS", getFlywheelRPS());
        panelsGraph.addData("Flywheel Error", getFlywheelErrorRPS());
    }

    public double getFlywheelErrorRPS() {
        return getFlywheelRPS() - desiredFlywheelRPS;
    }

    public double getFlywheelRPS() {
        return flywheel.getVelocity() / GOBILDA_5203_6000RPM;
    }

    public double getFeederPower() {
        return feeder.getPower();
    }

    public void speedUpFlywheel() {
        desiredFlywheelRPS = Math.min(desiredFlywheelRPS + 1, 100);
    }

    public void slowDownFlywheel() {
        desiredFlywheelRPS = Math.max(desiredFlywheelRPS - 1, 0);
    }

    public void toggleFlywheel() {
        canSpinFlywheel = !canSpinFlywheel;
    }

    public void feed() {
        requestedFeed = true;
    }

    public void feed(boolean reverse) {
        requestedFeed = true;
        requestedReverseFeed = true;
    }

    public void reverseFeed() {
        requestedFeed = true;
        requestedReverseFeed = true;
    }
}
