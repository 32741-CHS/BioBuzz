package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.SERVO_BIG_BALL;
import static org.firstinspires.ftc.teamcode.configs.Globals.SERVO_SMALL_BALL;
import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;
import org.firstinspires.ftc.teamcode.subsystems.GoalTrackerSimple;
import org.firstinspires.ftc.teamcode.subsystems.OdometryLogger;
import org.firstinspires.ftc.teamcode.subsystems.SimpleFeederAndFlywheel;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemHandler;

@TeleOp(name = "Simple Shooter Light TeleOp")
public class SimpleShooterTeleOp extends OpMode {
    private GamepadWrapper driverGamepad, shooterGamepad;
    private RobotHardwareGroup robotHardware;
    private SubsystemHandler allSubsystems;
    private Drivetrain drivetrain;
    private GoalTrackerSimple goalTrackerSimple;
    private SimpleFeederAndFlywheel simpleFeederAndFlywheel;


    @Override
    public void init() {
        // Robot Hardware Special Init
        (robotHardware = new RobotHardwareGroup()).init(hardwareMap);

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

        allSubsystems.init(robotHardware);

        /// Motor Configs
        robotHardware.intake.setDirection(DcMotorSimple.Direction.FORWARD);
        /// Key Binds
        driverGamepad.leftTrigger
                .onPress(() -> drivetrain.speed(0.5))
                .onRelease(() -> drivetrain.speed(1));

        shooterGamepad.leftTrigger
                .whenHeld(simpleFeederAndFlywheel::requestFeed)
                .onRelease(simpleFeederAndFlywheel::stopFeed);

        shooterGamepad.rightTrigger
                .onPress(() -> robotHardware.intake.setPower(1))
                .onRelease(() -> robotHardware.intake.setPower(0));

        shooterGamepad.leftBumper
                .onPress(() -> robotHardware.servo.setPosition(SERVO_SMALL_BALL))
                .onRelease(() -> robotHardware.servo.setPosition(SERVO_BIG_BALL));

    }

    @Override
    public void loop() {
        allSubsystems.update();

        // Telemetry
        panelsTelemetry.addData("[drivetrain] X", drivetrain.getX());
        panelsTelemetry.addData("[drivetrain] Y", drivetrain.getY());

        panelsTelemetry.addData(
                "[drivetrain] Heading",
                Math.toDegrees(drivetrain.getHeading())
        );

    }
}