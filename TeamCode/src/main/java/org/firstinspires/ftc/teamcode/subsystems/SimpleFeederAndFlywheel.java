package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.Globals.FEEDER_POWER;
import static org.firstinspires.ftc.teamcode.configs.Globals.FLYWHEEL_ERROR_TOL;
import static org.firstinspires.ftc.teamcode.configs.Globals.GOBILDA_5203_6000RPM;
import static org.firstinspires.ftc.teamcode.configs.Globals.SERVO_BIG_BALL;
import static org.firstinspires.ftc.teamcode.configs.Globals.SERVO_SMALL_BALL;
import static org.firstinspires.ftc.teamcode.configs.Globals.simpleTelemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configs.BotHardware;
import org.firstinspires.ftc.teamcode.utils.Ballistics;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

public class SimpleFeederAndFlywheel implements SubsystemBase {
    public boolean isFlywheelIdled = false;
    GoalTrackerSimple goalTrackerSimple;
    private DcMotorEx flywheel;
    private DcMotor feeder;
    private Servo servo;

    public SimpleFeederAndFlywheel(GoalTrackerSimple gts) {
        goalTrackerSimple = gts;
    }

    @Override
    public void init(BotHardware botHardware) {
        flywheel = (DcMotorEx) botHardware.flywheel;
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        feeder = botHardware.feeder;
        feeder.setDirection(DcMotorSimple.Direction.FORWARD);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = botHardware.servo;
    }


    @Override
    public void update() {
        flywheel.setVelocity((!isFlywheelIdled) ? desiredRPS() * GOBILDA_5203_6000RPM : 0);

        simpleTelemetry.addData("Flywheel RPS", getFlywheelRPS());
        simpleTelemetry.addData("Flywheel Error", getFlywheelErrorRPS());
    }

    double desiredRPS() {
        return Ballistics.calculateFlywheelRPS(goalTrackerSimple.getDistance());
    }

    double getFlywheelRPS() {
        return flywheel.getVelocity() / GOBILDA_5203_6000RPM;
    }

    double getFlywheelErrorRPS() {
        return getFlywheelRPS() - desiredRPS();
    }

    public void requestReversedFeed() {
        feeder.setPower(-1);
    }

    public void requestFeed() {
        if (Math.abs(getFlywheelErrorRPS()) <= FLYWHEEL_ERROR_TOL)
            feeder.setPower(FEEDER_POWER);
        else feeder.setPower(0);
    }

    public void stopFeed() {
        feeder.setPower(0);
    }

    public void runFlywheel() {
        isFlywheelIdled = false;
    }

    public void idleFlywheel() {
        isFlywheelIdled = true;
    }

    public void servoToBigBall() {
        servo.setPosition(SERVO_BIG_BALL);
    }

    public void servoToSmallBall() {
        servo.setPosition(SERVO_SMALL_BALL);
    }

}
