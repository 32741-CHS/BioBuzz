package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.simpleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.configs.Globals;
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
        simpleTelemetry.registerFtcTelemetry(telemetry);
        robotHardware = new RobotHardwareGroup(hardwareMap);

        // Pass all subsystems into the handler
        allSubsystems = new SubsystemHandler(
                (driverGamepad = GamepadWrapper.newDriverGamepad(gamepad1)),
                (shooterGamepad = GamepadWrapper.newOperatorGamepad(gamepad2)),
                (drivetrain = new Drivetrain(0, 0, 0)),
                new OdometryLogger()
        );

        drivetrain.bindToGamepad(driverGamepad); // Passing gamepad should be avoided as much as possible...
        allSubsystems.init(robotHardware);

        robotHardware.feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        // Key Binds
        driverGamepad.leftTrigger /// Speed Modifier
                .onPress(() -> drivetrain.speed(0.5))
                .onRelease(() -> drivetrain.speed(1));

        shooterGamepad.leftTrigger /// Feeder Toggle
                .onPress(() -> robotHardware.feeder.setPower(1))
                .onRelease(() -> robotHardware.feeder.setPower(0));

        shooterGamepad.rightTrigger /// Intake Toggle
                .onPress(() -> robotHardware.intake.setPower(1))
                .onRelease(() -> robotHardware.intake.setPower(0));

        shooterGamepad.btnB /// Flywheel Manual Power Modifier
                .onPress(() -> Globals.FLYWHEEL_POWER += 0.01);
        shooterGamepad.btnX
                .onRelease(() -> Globals.FLYWHEEL_POWER -= 0.01);

        shooterGamepad.leftBumper /// Servo Position
                .onPress(() -> robotHardware.servo.setPosition(
                        robotHardware.servo.getPosition() == Globals.SERVO_BIG_BALL ?
                                Globals.SERVO_SMALL_BALL : Globals.SERVO_BIG_BALL));

    }

    @Override
    public void loop() {
        robotHardware.flywheel.setPower(Globals.FLYWHEEL_POWER);

        allSubsystems.update();

        // Telemetry
        simpleTelemetry.addData("[drivetrain] X", drivetrain.getX());
        simpleTelemetry.addData("[drivetrain] Y", drivetrain.getY());

        simpleTelemetry.addData("[drivetrain] Heading DEGREES", Math.toDegrees(drivetrain.getHeading()));
        simpleTelemetry.addData("FLYWHEEL_POWER", Globals.FLYWHEEL_POWER);

        simpleTelemetry.addData("Servo Pos", robotHardware.servo.getPosition());

        simpleTelemetry.update();
    }

}