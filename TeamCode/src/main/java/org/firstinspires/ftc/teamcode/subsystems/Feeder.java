package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.Globals.FEEDER_POWER;
import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.BallType;
import org.firstinspires.ftc.teamcode.utils.MotorBasicActions;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

@Configurable
public class Feeder implements SubsystemBase, MotorBasicActions {
    public static double NECTAR_SERVO_POS = 0.0;
    public static double POLLEN_SERVO_POS = 0.45;
    private final Flywheel flywheel;
    private final ColourDetector colourDetector;
    private double desiredPower = 0;
    private boolean reverseMode = false;
    private DcMotor feeder;
    private Servo servo;
    private WebcamName insideCam;
    private BallType currentBallType = BallType.Big;

    public Feeder(Flywheel flywheel, ColourDetector colourDetector) {
        this.flywheel = flywheel;
        this.colourDetector = colourDetector;
    }

    @Override
    public void init(RobotHardwareGroup robotHardware) {
        feeder = robotHardware.feeder;
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        servo = robotHardware.servo;
        insideCam = robotHardware.insideCam;
        applyServoPosition();
    }

    @Override
    public void update() {
        if (colourDetector == null && insideCam == null) return;
        BallType detectedType = colourDetector.getBallType(insideCam);
        if (detectedType != BallType.None) {
            setBallType(detectedType);
        }

        feeder.setPower(desiredPower);
        applyServoPosition();

        panelsTelemetry.addData("Feeder Power", reverseMode ? -FEEDER_POWER : FEEDER_POWER);
        panelsTelemetry.addData("Flywheel ready to feed?", flywheel.isReadyForFeed());
        panelsTelemetry.addData("Ball Type", currentBallType.name());
        if (servo != null) {
            panelsTelemetry.addData("Pusher Servo Position", servo.getPosition());
        }
    }

    public void setBallType(BallType type) {
        this.currentBallType = type;
        applyServoPosition();
    }

    private void applyServoPosition() {
        if (servo == null) return;
        double targetPos = (currentBallType == BallType.Small) ? POLLEN_SERVO_POS : NECTAR_SERVO_POS;
        servo.setPosition(targetPos);
    }

    public BallType getCurrentBallType() {
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
