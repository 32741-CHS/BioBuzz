package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;
import org.firstinspires.ftc.teamcode.subsystems.OdometryLogger;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemHandler;

@TeleOp(name = "Light TeleOp")
public class LightTeleOp extends OpMode {
    GamepadWrapper driverGamepad, shooterGamepad;
    private RobotHardwareGroup robotHardware;
    private SubsystemHandler allSubsystems;
    private Drivetrain drivetrain;

    @Override
    public void init() {
        // Robot Hardware Special Init
        (robotHardware = new RobotHardwareGroup()).init(hardwareMap);
        // Pass all subsystems into the handler
        allSubsystems = new SubsystemHandler(
                (driverGamepad = GamepadWrapper.newDriverGamepad(gamepad1)),
                (shooterGamepad = GamepadWrapper.newOperatorGamepad(gamepad2)),
                (drivetrain = new Drivetrain(0, 0, 0)),
                new OdometryLogger()
        );

        drivetrain.bindToGamepad(driverGamepad); // Passing gamepad should be avoided as much as possible...

        allSubsystems.init(robotHardware);

        robotHardware.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.flywheel.setPower(0.65);

        // Key Binds
        driverGamepad.leftTrigger
                .onPress(() -> drivetrain.speed(0.5))
                .onRelease(() -> drivetrain.speed(1));

        shooterGamepad.leftTrigger
                .onPress(() -> robotHardware.feeder.setPower(1))
                .onRelease(() -> robotHardware.feeder.setPower(0));

        shooterGamepad.rightTrigger
                .onPress(() -> robotHardware.intake.setPower(-1))
                .onRelease(() -> robotHardware.intake.setPower(0));

        shooterGamepad.leftBumper
                .whenHeld(() -> robotHardware.servo.setPosition(Math.max(robotHardware.servo.getPosition() - 0.01, 0)));

        shooterGamepad.rightBumper
                .whenHeld(() -> robotHardware.servo.setPosition(Math.min(robotHardware.servo.getPosition() + 0.01, 1)));
    }

    @Override
    public void loop() {
        allSubsystems.update();

        // Telemetry
        log("[drivetrain] X", drivetrain.getX());
        log("[drivetrain] Y", drivetrain.getY());

        log(
                "[drivetrain] Heading",
                Math.toDegrees(drivetrain.getHeading())
        );
        
        log("Servo Pos", robotHardware.servo.getPosition());

        panelsTelemetry.update();
        telemetry.update();
    }

    void log(String key, double value) {
        panelsTelemetry.addData(key, value);
        telemetry.addData(key, value);
    }
}