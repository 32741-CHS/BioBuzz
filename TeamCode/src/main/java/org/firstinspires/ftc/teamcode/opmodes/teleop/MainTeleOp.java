package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.configs.Globals.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.configs.Globals.RED_GOAL;
import static org.firstinspires.ftc.teamcode.configs.Globals.STICK_DEADBAND;
import static org.firstinspires.ftc.teamcode.configs.Globals.robotHardwareGroup;

import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Ballistics;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Locale;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends OpMode {
    public static boolean isRed = false;
    public static boolean useFlywheelLookups = true;
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final GraphManager panelsGraph = PanelsGraph.INSTANCE.getManager();
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private Vision vision;
    private boolean isFieldDriving = false;
    private boolean turretManualMode = false;
    private double manualTurretAngle = 0;
    private double lastGoalTagTime = 0;

    @Override
    public void init() {
        // hardwareMap, gamepad1, and gamepad2 are magically injected here
        // by ftc for us to use. We wrap them so simple functions for us.
        robotHardwareGroup.init(hardwareMap);
        driverGamepad = GamepadEx.newDriverGamepad(gamepad1);
        operatorGamepad = GamepadEx.newOperatorGamepad(gamepad2);

        drivetrain = new Drivetrain();
        intake = new Intake();
        shooter = new Shooter();
        vision = new Vision();
        turret = new Turret();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", "Square = Red, X = Blue");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        driverGamepad.update();
        if (driverGamepad.btnX.isHeld()) isRed = true;
        if (driverGamepad.btnA.isHeld()) isRed = false;

        telemetry.addData("Team", isRed ? "RED" : "BLUE");
        telemetry.addData("Switch", "Square = Red, X = Blue");
        telemetry.update();
    }

    @Override
    public void start() {
        lastGoalTagTime = getRuntime();

        // KEY BINDS
        // Notes:
        // - whenHeld means run EVERY frame
        // - OnPress when the key goes down, fire the function ONCE
        // - The "::" means don't call this function, just pass it forward as a parameter
        driverGamepad.leftBumper.whenHeld(() -> drivetrain.setSpeedMultiplier(1));
        driverGamepad.leftBumper.whenNotHeld(() -> drivetrain.setSpeedMultiplier(0.1));
        operatorGamepad.dpadUp.onPress(shooter::speedUpFlywheel);
        operatorGamepad.dpadDown.onPress(shooter::slowDownFlywheel);
        operatorGamepad.btnY.onPress(drivetrain::resetIMU);
        operatorGamepad.btnB.onPress(turret::resetTurretEncoder);
        operatorGamepad.leftBumper.whenHeld(intake::eat);
        operatorGamepad.btnA.whenHeld(intake::invert);
        operatorGamepad.rightTrigger.whenHeld(shooter::feed);
        operatorGamepad.btnA.whenHeld(shooter::reverseFeed);
        operatorGamepad.btnX.onPress(shooter::toggleFlywheel);
        driverGamepad.btnY.onPress(() -> isFieldDriving = !isFieldDriving);
        operatorGamepad.leftBumper.onPress(() -> useFlywheelLookups = !useFlywheelLookups);
    }

    @Override
    public void loop() {
        driverGamepad.update();
        operatorGamepad.update();

        // TODO: replace with follower.setTeleOpDrive() once Pedro Pathing is added
        drivetrain.drive(
                -driverGamepad.raw.left_stick_y,
                driverGamepad.raw.left_stick_x,
                driverGamepad.raw.right_stick_x,
                isFieldDriving);

        // turret: right stick x for manual override, or auto-track the goal tag
        double stickX = operatorGamepad.raw.right_stick_x;
        if (Math.abs(stickX) > STICK_DEADBAND) {
            if (!turretManualMode) {
                turretManualMode = true;
            }
        } else {
            turretManualMode = false;
        }

        AprilTagDetection goalTag = vision.getTagById(isRed ? RED_GOAL : BLUE_GOAL);

        if (turretManualMode) {
            lastGoalTagTime = getRuntime();
            manualTurretAngle = turret.getCurrentAngle();
            manualTurretAngle += stickX * 3;
            turret.goTo(manualTurretAngle);
        } else {
            if (goalTag != null) {
                lastGoalTagTime = getRuntime();
                double bearing = Math.toDegrees(goalTag.ftcPose.bearing);
                double angle = Ballistics.calculateTurretAngle(bearing, turret.getCurrentAngle());
                turret.goTo(angle);
            } else if (getRuntime() - lastGoalTagTime > Turret.LOST_TAG_RETURN_DELAY) {
                turret.returnHome();
            }
        }

        if (useFlywheelLookups && goalTag != null) {
            double distance = goalTag.ftcPose.range;
            Shooter.setDesiredFlywheelRPS(Ballistics.calculateFlywheelRPS(distance));
        }

        intake.update();
        shooter.update();
        turret.update();

        // Telemetry
        panelsTelemetry.addData("Intake power", intake.getPower());
        panelsTelemetry.addData("Feeder power", shooter.getFeederPower());
        panelsTelemetry.addData("Flywheel rps", shooter.getFlywheelRPS());
        panelsTelemetry.addData("Flywheel error", shooter.getFlywheelErrorRPS());
        panelsTelemetry.addData("Drivetrain speed", drivetrain.getSpeedMultiplier());
        panelsTelemetry.addData("Turret angle", turret.getCurrentAngle());
        panelsTelemetry.addData("Turret error", turret.getErrorAngle());
        panelsTelemetry.addData("Turret mode", turretManualMode ? "MANUAL" : "AUTO");
        panelsTelemetry.addData("Field Centric", isFieldDriving);
        panelsTelemetry.addData("Use lookups", useFlywheelLookups);

        if (goalTag != null) {
            // Locale.US means use the 12.22 every time instead of 12,22 because some countries use
            // the "," instead of the "." why I do not know.
            panelsTelemetry.addData(
                    "Tag distance", String.format(Locale.US, "%.2f m", goalTag.ftcPose.range));
            panelsTelemetry.addData(
                    "Tag bearing",
                    String.format(Locale.US, "%.1f deg", Math.toDegrees(goalTag.ftcPose.bearing)));
        } else {
            panelsTelemetry.addData("Tag distance", "no tag");
        }

        // panels graph feed
        panelsGraph.addData("flywheelRPS", shooter.getFlywheelRPS());
        panelsGraph.addData("flywheelTarget", Shooter.desiredFlywheelRPS);
        panelsGraph.addData("flywheelError", shooter.getFlywheelErrorRPS());
        panelsGraph.addData("feederPower", shooter.getFeederPower());
        panelsGraph.addData("intakePower", intake.getPower());

        panelsGraph.update();
        panelsTelemetry.update(telemetry);
    }
}
