package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.SERVO_BIG_BALL;
import static org.firstinspires.ftc.teamcode.configs.Globals.SERVO_SMALL_BALL;
import static org.firstinspires.ftc.teamcode.configs.Globals.simpleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.configs.BotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;
import org.firstinspires.ftc.teamcode.subsystems.GoalTrackerSimple;
import org.firstinspires.ftc.teamcode.subsystems.OdometryLogger;
import org.firstinspires.ftc.teamcode.subsystems.SimpleFeederAndFlywheel;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemHandler;

@TeleOp(name = "Simple Shooter Light TeleOp")
public class SimpleShooterTeleOp extends OpMode {
    private GamepadWrapper driverGamepad, shooterGamepad;
    private BotHardware botHardware;
    private SubsystemHandler allSubsystems;
    private Drivetrain drivetrain;
    private GoalTrackerSimple goalTrackerSimple;
    private SimpleFeederAndFlywheel simpleFeederAndFlywheel;
    private double heading;


    @Override
    public void init() {
        // Robot Hardware Special Init
        simpleTelemetry.registerFtcTelemetry(telemetry);
        botHardware = new BotHardware(hardwareMap);

        // Pass all subsystems into the handler
        allSubsystems = new SubsystemHandler(
                (driverGamepad = GamepadWrapper.newDriverGamepad(gamepad1)),
                (shooterGamepad = GamepadWrapper.newOperatorGamepad(gamepad2)),
                (drivetrain = new Drivetrain(0, 0, 0)),
                (goalTrackerSimple = new GoalTrackerSimple()),
                (simpleFeederAndFlywheel = new SimpleFeederAndFlywheel(goalTrackerSimple)),
                new OdometryLogger()
        );

        drivetrain.bindToGamepad(driverGamepad); // Passing gamepad should be avoided as much as possible...

        allSubsystems.init(botHardware);

        /// Motor Configs
        botHardware.intake.setDirection(DcMotorSimple.Direction.FORWARD);
        /// Key Binds
        driverGamepad.leftTrigger
                .onPress(() -> drivetrain.speed(0.5))
                .onRelease(() -> drivetrain.speed(1));

        shooterGamepad.leftTrigger
                .whenHeld(simpleFeederAndFlywheel::requestFeed)
                .onRelease(simpleFeederAndFlywheel::stopFeed);

        shooterGamepad.rightTrigger
                .onPress(() -> botHardware.intake.setPower(1))
                .onRelease(() -> botHardware.intake.setPower(0));

        shooterGamepad.leftBumper
                .onPress(() -> botHardware.servo.setPosition(SERVO_SMALL_BALL))
                .onRelease(() -> botHardware.servo.setPosition(SERVO_BIG_BALL));

        driverGamepad.rightBumper
                .whenHeld(
                        () -> drivetrain.turnInput(heading)
                );

    }

    @Override
    public void loop() {
        allSubsystems.update();
        heading = goalTrackerSimple.getXHeading();

        // Telemetry
        simpleTelemetry.addData("[drivetrain] X", drivetrain.getX());
        simpleTelemetry.addData("[drivetrain] Y", drivetrain.getY());

        simpleTelemetry.addData(
                "[drivetrain] Heading",
                Math.toDegrees(drivetrain.getHeading())
        );
        simpleTelemetry.addData("G TRACK", goalTrackerSimple.getXHeading());

    }
}