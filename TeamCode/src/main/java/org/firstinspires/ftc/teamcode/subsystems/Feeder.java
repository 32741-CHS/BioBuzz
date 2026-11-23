package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.Globals.FEEDER_POWER;
import static org.firstinspires.ftc.teamcode.configs.Globals.simpleTelemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configs.BotHardware;
import org.firstinspires.ftc.teamcode.configs.Globals;
import org.firstinspires.ftc.teamcode.utils.MotorBasicActions;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

@Configurable
public class Feeder implements SubsystemBase, MotorBasicActions {
    public static double NECTAR_SERVO_POS = 0.0;
    public static double POLLEN_SERVO_POS = 0.45;
    private final Flywheel flywheel;
    // private final ColourDetector colourDetector;
    private double desiredPower = 0;
    private boolean reverseMode = false;
    private DcMotor feeder;
    private Servo servo;
    // private WebcamName insideCam;
    private Globals.BallType currentBallType = Globals.BallType.Big;

    public Feeder(Flywheel flywheel/*, ColourDetector colourDetector*/) {
        this.flywheel = flywheel;
        // this.colourDetector = colourDetector;
    }

    @Override
    public void init(BotHardware botHardware) {
        feeder = botHardware.feeder;
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        servo = botHardware.servo;
        // insideCam = botHardware.insideCam;
        applyServoPosition();
    }

    @Override
    public void update() {
        /*
        if (colourDetector == null && insideCam == null) return;
        Globals.BallType detectedType = colourDetector.getBallType(insideCam);
        if (detectedType != Globals.BallType.None) {
            setBallType(detectedType);
        }
        */

        feeder.setPower(desiredPower);
        applyServoPosition();

        simpleTelemetry.addData("Feeder Power", reverseMode ? -FEEDER_POWER : FEEDER_POWER);
        simpleTelemetry.addData("Flywheel ready to feed?", flywheel.isReadyForFeed());
        simpleTelemetry.addData("Ball Type", currentBallType.name());
        if (servo != null) {
            simpleTelemetry.addData("Pusher Servo Position", servo.getPosition());
        }
    }

    private void applyServoPosition() {
        if (servo == null) return;
        double targetPos = (currentBallType == Globals.BallType.Small) ? POLLEN_SERVO_POS : NECTAR_SERVO_POS;
        servo.setPosition(targetPos);
    }

    public void setBallType(Globals.BallType type) {
        this.currentBallType = type;
        applyServoPosition();
    }

    public Globals.BallType getCurrentBallType() {
        return currentBallType;
    }

    public void tryRunMotorIfReady() {
        if (flywheel.isReadyForFeed()) {
            /// Make servo go to correct position based on ball size
            desiredPower = reverseMode ? -FEEDER_POWER : FEEDER_POWER;
        }
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
