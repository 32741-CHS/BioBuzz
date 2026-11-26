package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.simpleTelemetry;

import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configs.BotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;
import org.firstinspires.ftc.teamcode.subsystems.GoalTracker;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
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
    private Intake intake;
    private Feeder feeder;
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
        allSubsystems = new SubsystemHandler(
                // (colourDetector = new ColourDetector()),
                (driverGamepad = GamepadWrapper.newDriverGamepad(gamepad1)),
                (shooterGamepad = GamepadWrapper.newOperatorGamepad(gamepad2)),
                (drivetrain = new Drivetrain(0, 0, 0)),

                (turret = new Turret(drivetrain)),
                (flywheel = new Flywheel(drivetrain/*, colourDetector*/)),
                (feeder = new Feeder(flywheel/*, colourDetector*/)),
                (goaltracker = new GoalTracker(turret, flywheel))
        );

        drivetrain.bindToGamepad(driverGamepad); // Passing gamepad should be avoided as much as possible...

        allSubsystems.init(botHardware);
        Flywheel.canSpinFlywheel = true;

        // Key Binds
        driverGamepad.leftTrigger /// Robot Speed
                .onPress(() -> drivetrain.speed(0.5))
                .onRelease(() -> drivetrain.speed(1));
//        driverGamepad.btnY
//                .onPress(() -> fieldCentricDriving = !fieldCentricDriving);

        shooterGamepad.leftTrigger
                .onPress(() -> botHardware.intake.setPower(1))
                .onRelease(() -> botHardware.intake.setPower(0));

        shooterGamepad.rightTrigger
                .onPress(() -> botHardware.feeder.setPower(1))
                .onRelease(() -> botHardware.feeder.setPower(0));

        shooterGamepad.btnA /// Reverse
                .onPress(() -> botHardware.intake.setPower(-1))
                .onPress(() -> botHardware.intake.setPower(-1))
                .onRelease(() -> botHardware.feeder.setPower(0))
                .onRelease(() -> botHardware.feeder.setPower(0));
        /*
        shooterGamepad.rightTrigger /// Feed Ball (Shoot)
                .whenHeld(feeder::tryRunMotorIfReady)
                .onRelease(feeder::stopMotor);
         */
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

        simpleTelemetry.update();
    }
}