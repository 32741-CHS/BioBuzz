package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.simpleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configs.BotHardware;
import org.firstinspires.ftc.teamcode.configs.StandardKeyBinds;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;
import org.firstinspires.ftc.teamcode.subsystems.GoalTrackerSimple;
import org.firstinspires.ftc.teamcode.subsystems.SimpleFeederAndFlywheel;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemHandler;

@TeleOp(name = "Simple Shooter Light TeleOp")
public class SimpleShooterTeleOp extends OpMode {
    private BotHardware botHardware;
    private SubsystemHandler allSubsystems;
    private Drivetrain drivetrain;
    private GoalTrackerSimple goalTrackerSimple;
    private SimpleFeederAndFlywheel simpleFeederAndFlywheel;
    private Flywheel fancyFlywheel;


    @Override
    public void init() {
        // Robot Hardware Special Init
        simpleTelemetry.registerFtcTelemetry(telemetry);
        botHardware = new BotHardware(hardwareMap);

        // Pass all subsystems into the handler
        GamepadWrapper driverGamepad, shooterGamepad;

        allSubsystems = new SubsystemHandler(
                (driverGamepad = GamepadWrapper.newDriverGamepad(gamepad1)),
                (shooterGamepad = GamepadWrapper.newOperatorGamepad(gamepad2)),
                (drivetrain = new Drivetrain(0, 0, 0)),
                (goalTrackerSimple = new GoalTrackerSimple()),
                // (simpleFeederAndFlywheel = new SimpleFeederAndFlywheel(goalTrackerSimple)),
                (fancyFlywheel = new Flywheel())
        );

        drivetrain.bindToGamepad(driverGamepad); // Passing gamepad should be avoided as much as possible...

        allSubsystems.init(botHardware);

        /// Key Binds
        new StandardKeyBinds(botHardware, drivetrain, driverGamepad, shooterGamepad);
    }

    @Override
    public void loop() {
        allSubsystems.update();

        fancyFlywheel.changeGoal(goalTrackerSimple.getDistance(), goalTrackerSimple.getZHeading());

        drivetrain.turnInput(goalTrackerSimple.getXHeading());

        /// Telemetry
        simpleTelemetry.addData("[GoalTrackerSimple] getDistance", goalTrackerSimple.getDistance());
        simpleTelemetry.addData("[GoalTrackerSimple] getXHeading", goalTrackerSimple.getXHeading());

    }
}