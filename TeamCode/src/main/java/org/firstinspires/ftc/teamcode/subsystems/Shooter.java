package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.TickRates.GOBILDA_5203_6000RPM;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.configs.RobotHardware;

@Configurable
public class Shooter {
    private final DcMotorEx flywheel;
    private final DcMotor feeder;

    public static double desiredFlywheelRPS = 13;
    public static double desiredFeederPower = 0.7;

    public static double flywheelKP = 16.5;
    public static double flywheelKF = 13.5;

    public static double FLYWHEEL_ERROR_TOL = 1.0;

    public static boolean canSpinFlywheel = false;

    private boolean requestedFeed = false;
    private boolean requestedReverseFeed = false;

    public Shooter(RobotHardware hw) {
        flywheel = hw.flywheel;
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        feeder = hw.feeder;
        feeder.setDirection(DcMotorSimple.Direction.FORWARD);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public double getFlywheelErrorRPS() {
        return getFlywheelRPS() - desiredFlywheelRPS;
    }

    public void toggleFlywheel() {
        canSpinFlywheel = !canSpinFlywheel;
    }

    public static void setDesiredFlywheelRPS(double rps) {
        desiredFlywheelRPS = rps;
    }

    public void feed() {
        requestedFeed = true;
    }

    public void reverseFeed() {
        requestedFeed = true;
        requestedReverseFeed = true;
    }

    public void update() {
        // apply pidf every frame so @Configurable changes work
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(flywheelKP, 0, 0, flywheelKF));

        flywheel.setVelocity(canSpinFlywheel ? desiredFlywheelRPS * GOBILDA_5203_6000RPM : 0);

        if (requestedFeed && Math.abs(getFlywheelErrorRPS()) <= FLYWHEEL_ERROR_TOL) {
            feeder.setPower(requestedReverseFeed ? -desiredFeederPower : desiredFeederPower);
        } else {
            feeder.setPower(0);
        }
        requestedFeed = false;
        requestedReverseFeed = false;
    }
}
