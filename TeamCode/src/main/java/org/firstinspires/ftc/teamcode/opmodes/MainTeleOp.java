package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.OdometryLogger;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemHandler;

@TeleOp(name = "BioBuzz TeleOp")
public class MainTeleOp extends OpMode {
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // EXAMPLE starting position. Replace with your actual position.
    private final Pose startPose = poseFactory.of(24, 24, 0);
    GamepadWrapper driverGamepad, shooterGamepad;
    private RobotHardwareGroup robotHardware;
    private SubsystemHandler allSubsystems;
    private Drivetrain drivetrain;
    private OdometryLogger pinpointLogger;
    private Intake intake;
    private Feeder feeder;
    private Flywheel flywheel;

    @Override
    public void init() {
        // Robot Hardware Special Init
        (robotHardware = new RobotHardwareGroup()).init(hardwareMap);

        // Subsystems
        flywheel = new Flywheel();
        feeder = new Feeder(flywheel);
        intake = new Intake();
        pinpointLogger = new OdometryLogger();
        driverGamepad = GamepadWrapper.newDriverGamepad(gamepad1);
        shooterGamepad = GamepadWrapper.newOperatorGamepad(gamepad2);
        drivetrain = new Drivetrain(startPose);
        drivetrain.bindToGamepad(driverGamepad); // Passing gamepad should be avoided as much as possible...

        // Pass all subsystems into the handler
        allSubsystems = new SubsystemHandler(
                flywheel,
                intake,
                feeder,
                pinpointLogger,
                drivetrain,
                driverGamepad,
                shooterGamepad
        );


        allSubsystems.init(robotHardware);

        // Key Binds
        driverGamepad.leftTrigger
                .onPress(() -> drivetrain.speed(0.5))
                .onRelease(() -> drivetrain.speed(1));
//        driverGamepad.btnY
//                .onPress(() -> fieldCentricDriving = !fieldCentricDriving);

        shooterGamepad.leftTrigger
                .onPress(intake::run)
                .onRelease(intake::stopRunning);

        shooterGamepad.btnA
                .onPress(intake::reverse)
                .onPress(feeder::reverse)
                .onRelease(intake::unReverse)
                .onRelease(feeder::unReverse);

        shooterGamepad.rightTrigger
                .whenHeld(feeder::tryRun)
                .onRelease(feeder::stopRunning);
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

        panelsTelemetry.update();
    }
}