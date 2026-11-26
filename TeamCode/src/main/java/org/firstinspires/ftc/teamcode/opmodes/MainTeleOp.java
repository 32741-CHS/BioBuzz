package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.simpleTelemetry;

import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configs.BotHardware;
import org.firstinspires.ftc.teamcode.configs.StandardKeyBinds;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;
import org.firstinspires.ftc.teamcode.subsystems.GoalTracker;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemHandler;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "(Main) BioBuzz TeleOp")
public class MainTeleOp extends OpMode {
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // EXAMPLE starting position. Replace with your actual position.
    GamepadWrapper driverGamepad, shooterGamepad;
    private BotHardware botHardware;
    private SubsystemHandler allSubsystems;
    private Drivetrain drivetrain;
    private Flywheel flywheel;
    // private ColourDetector colourDetector;
    private Turret turret;
    private GoalTracker goaltracker;

    @Override
    public void init() {
        simpleTelemetry.registerFtcTelemetry(telemetry);
        botHardware = new BotHardware(hardwareMap);

        // Subsystems

        // Pass all subsystems into the handler
        Feeder feeder;
        allSubsystems = new SubsystemHandler(
                // (colourDetector = new ColourDetector()),
                (driverGamepad = GamepadWrapper.newDriverGamepad(gamepad1)),
                (shooterGamepad = GamepadWrapper.newOperatorGamepad(gamepad2)),
                (drivetrain = new Drivetrain(0, 0, 0)),

                (turret = new Turret(drivetrain)),
                (flywheel = new Flywheel(/*, colourDetector*/)),
                (feeder = new Feeder(flywheel/*, colourDetector*/)),
                (goaltracker = new GoalTracker())
        );

        drivetrain.bindToGamepad(driverGamepad); // Passing gamepad should be avoided as much as possible...

        allSubsystems.init(botHardware);

        /// Key Binds
        new StandardKeyBinds(botHardware, drivetrain, driverGamepad, shooterGamepad);
    }

    @Override
    public void loop() {
        allSubsystems.update();

        // Telemetry
        simpleTelemetry.addData("[drivetrain] X", drivetrain.getX());
        simpleTelemetry.addData("[drivetrain] Y", drivetrain.getY());

        simpleTelemetry.addData(
                "[drivetrain] Heading",
                Math.toDegrees(drivetrain.getHeading())
        );

        // Flywheel & Turret
        if (goaltracker.getBest() != null) {
            double distanceToGoal = goaltracker.getDistance();
            double xRadians = goaltracker.getXRadians();
            double zRadians = goaltracker.getZRadians();
            if (turret != null) {
                turret.changeGoal(distanceToGoal, xRadians, zRadians);
            }
            if (flywheel != null) {
                flywheel.changeGoal(distanceToGoal, zRadians);
            }
        }
    }
}