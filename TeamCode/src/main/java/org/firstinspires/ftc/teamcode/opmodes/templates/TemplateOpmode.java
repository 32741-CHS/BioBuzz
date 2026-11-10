package org.firstinspires.ftc.teamcode.opmodes.templates;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemHandler;

@Disabled
@TeleOp(name = "BioBuzz TeleOp")
public class TemplateOpmode extends OpMode {
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // EXAMPLE starting position. Replace with your actual position.
    private final Pose startPose = poseFactory.of(24, 24, 0);
    GamepadEx driverGamepad, shooterGamepad;
    private Follower follower;
    private RobotHardwareGroup robotHardware;
    private SubsystemHandler allSubsystems;
    private Drivetrain drivetrain;

    @Override
    public void init() {
        // Robot Hardware Special Init
        (robotHardware = new RobotHardwareGroup()).init(hardwareMap);

        // Subsystems
        driverGamepad = GamepadEx.newDriverGamepad(gamepad1);
        shooterGamepad = GamepadEx.newOperatorGamepad(gamepad2);
        drivetrain = new Drivetrain(startPose);
        drivetrain.bindToGamepad(driverGamepad);

        // Pass all subsystems into the handler
        allSubsystems = new SubsystemHandler(
                drivetrain,
                driverGamepad,
                shooterGamepad
        );


        allSubsystems.init(robotHardware);
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

        telemetry.addData("Pedro Mode", follower.mode());

        telemetry.update();
    }
}