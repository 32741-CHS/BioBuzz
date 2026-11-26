package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.simpleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configs.BotHardware;
import org.firstinspires.ftc.teamcode.configs.Globals;
import org.firstinspires.ftc.teamcode.configs.StandardKeyBinds;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemHandler;

/// Light TeleOp is an almost entirely self-contained script designed for basic operations.
@TeleOp(name = "Light TeleOp")
public class LightTeleOp extends OpMode {
    GamepadWrapper driverGamepad, shooterGamepad;
    private BotHardware botHardware;
    private SubsystemHandler allSubsystems;
    private Drivetrain drivetrain;

    @Override
    public void init() {
        simpleTelemetry.registerFtcTelemetry(telemetry);
        botHardware = new BotHardware(hardwareMap);

        // Pass all subsystems into the handler
        allSubsystems = new SubsystemHandler(
                (driverGamepad = GamepadWrapper.newDriverGamepad(gamepad1)),
                (shooterGamepad = GamepadWrapper.newOperatorGamepad(gamepad2)),
                (drivetrain = new Drivetrain(0, 0, 0))
        );

        drivetrain.bindToGamepad(driverGamepad); // Passing gamepad should be avoided as much as possible...
        allSubsystems.init(botHardware);

        /// Key Binds
        new StandardKeyBinds(botHardware, drivetrain, driverGamepad, shooterGamepad);

        shooterGamepad.btnB /// Flywheel Manual Power Modifier
                .onPress(() -> Globals.FLYWHEEL_POWER += 0.01);
        shooterGamepad.btnX
                .onRelease(() -> Globals.FLYWHEEL_POWER -= 0.01);

    }

    @Override
    public void loop() {
        allSubsystems.update();

        botHardware.flywheel.setPower(Globals.FLYWHEEL_POWER);

        /// Telemetry
        simpleTelemetry.addData("[drivetrain] Heading DEGREES", Math.toDegrees(drivetrain.getHeading()));
        simpleTelemetry.addData("FLYWHEEL_POWER", Globals.FLYWHEEL_POWER);
    }

}