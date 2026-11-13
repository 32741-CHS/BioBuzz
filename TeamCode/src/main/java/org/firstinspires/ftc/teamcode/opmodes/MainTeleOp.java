package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;
import org.firstinspires.ftc.teamcode.subsystems.GoalTracker;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.OdometryLogger;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemHandler;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

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
    private Camera camera;
    private Turret turret;
    private GoalTracker goaltracker;

    @Override
    public void init() {
        // Robot Hardware Special Init
        (robotHardware = new RobotHardwareGroup()).init(hardwareMap);

        // Subsystems
        intake = new Intake();
        driverGamepad = GamepadWrapper.newDriverGamepad(gamepad1);
        shooterGamepad = GamepadWrapper.newOperatorGamepad(gamepad2);
        drivetrain = new Drivetrain(startPose);
        drivetrain.bindToGamepad(driverGamepad); // Passing gamepad should be avoided as much as possible...
        camera = new Camera(
                robotHardware.insideCam,
                robotHardware.vision
        );
        flywheel = new Flywheel(drivetrain, camera);
        turret = new Turret(drivetrain);
        goaltracker = new GoalTracker(turret, flywheel);
        feeder = new Feeder(flywheel, camera);

        // Pass all subsystems into the handler
        allSubsystems = new SubsystemHandler(
                driverGamepad,
                shooterGamepad,
                flywheel,
                turret,
                goaltracker,
                camera,
                intake,
                feeder,
                drivetrain
        );


        allSubsystems.init(robotHardware);

        // Key Binds
        driverGamepad.leftTrigger /// Robot Speed
                .onPress(() -> drivetrain.speed(0.5))
                .onRelease(() -> drivetrain.speed(1));
//        driverGamepad.btnY
//                .onPress(() -> fieldCentricDriving = !fieldCentricDriving);

        shooterGamepad.leftTrigger /// Intake
                .whenHeld(intake::tryRunMotorIfReady)
                .onRelease(intake::stopMotor);

        shooterGamepad.btnA /// Reverse ALL
                .onPress(intake::reverseMode)
                .onPress(feeder::reverseMode)
                .onRelease(intake::deactivateReverseMode)
                .onRelease(feeder::deactivateReverseMode);

        shooterGamepad.rightTrigger /// Feed Ball (Shoot)
                .whenHeld(feeder::tryRunMotorIfReady)
                .onRelease(feeder::stopMotor);
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