package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.fieldCentricDriving;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemHandler;

@Disabled
@TeleOp(name = "TemplateOpmode", group = "Tests")
public class TemplateOpmode extends OpMode {
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // EXAMPLE starting position. Replace with your actual position.
    private final Pose startPose = poseFactory.of(24, 24, 0);
    GamepadWrapper driverGamepad, shooterGamepad;
    private RobotHardwareGroup robotHardware;
    private SubsystemHandler allSubsystems;
    private Drivetrain drivetrain;

    @Override
    public void init() {
        // Robot Hardware Special Init
        (robotHardware = new RobotHardwareGroup()).init(hardwareMap);

        // Subsystems
        driverGamepad = GamepadWrapper.newDriverGamepad(gamepad1);
        shooterGamepad = GamepadWrapper.newOperatorGamepad(gamepad2);
        drivetrain = new Drivetrain(startPose);
        drivetrain.bindToGamepad(driverGamepad); // Passing gamepad should be avoided as much as possible...

        // Pass all subsystems into the handler
        allSubsystems = new SubsystemHandler(
                drivetrain,
                driverGamepad,
                shooterGamepad
        );

        allSubsystems.init(robotHardware);

        // Key Binds
        driverGamepad.leftBumper
                .onPress(() -> drivetrain.speed(0.5))
                .onRelease(() -> drivetrain.speed(1));
        driverGamepad.btnY
                .onPress(() -> fieldCentricDriving = !fieldCentricDriving);

//        shooterGamepad.leftBumper // Manual Flywheel Control
//                .whenHeld()
//                .whenNotHeld()
//        shooterGamepad.leftBumper.whenHeld(intake::eat);
//        shooterGamepad.btnA.whenHeld(intake::invert);
//        shooterGamepad.rightTrigger.whenHeld(shooter::feed);
//        shooterGamepad.btnA.whenHeld(shooter::reverseFeed);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        allSubsystems.update();

        // Telemetry
        telemetry.addData("[drivetrain] X", "%.2f", drivetrain.getX());
        telemetry.addData("[drivetrain] Y", "%.2f", drivetrain.getY());

        telemetry.addData(
                "[drivetrain] Heading",
                "%.2f°",
                Math.toDegrees(drivetrain.getHeading())
        );
        telemetry.update();
    }

}