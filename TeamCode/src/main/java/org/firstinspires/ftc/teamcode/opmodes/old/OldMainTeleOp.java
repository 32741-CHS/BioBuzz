package org.firstinspires.ftc.teamcode.opmodes.old;

import static org.firstinspires.ftc.teamcode.configs.Globals.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.configs.Globals.RED_GOAL;
import static org.firstinspires.ftc.teamcode.configs.Globals.STICK_DEADBAND;
import static org.firstinspires.ftc.teamcode.configs.Globals.simpleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;
import org.firstinspires.ftc.teamcode.subsystems.old.OldDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.old.OldIntake;
import org.firstinspires.ftc.teamcode.subsystems.old.OldShooter;
import org.firstinspires.ftc.teamcode.subsystems.old.OldTurret;
import org.firstinspires.ftc.teamcode.utils.Ballistics;
import org.firstinspires.ftc.teamcode.utils.Vector2D;
import org.firstinspires.ftc.teamcode.utils.old.OldVision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Locale;

@Disabled
@TeleOp(name = "Old Main TeleOp", group = "TeleOp")
public class OldMainTeleOp extends OpMode {
    public static boolean isRed = false;
    public static boolean useFlywheelLookups = true;
    private GamepadWrapper driverGamepad;
    private GamepadWrapper operatorGamepad;
    private OldDrivetrain oldDrivetrain;
    private OldIntake oldIntake;
    private OldShooter oldShooter;
    private OldTurret oldTurret;
    private OldVision oldVision;
    private RobotHardwareGroup robotHardware;
    private boolean isFieldDriving = false;
    private boolean turretManualMode = false;
    private double manualTurretAngle = 0;
    private double lastGoalTagTime = 0;

    @Override
    public void init() {
        // hardwareMap, gamepad1, and gamepad2 are magically injected here
        // by ftc for us to use. We wrap them so simple functions for us.

        // New robotHardware To make force fresh init and reduce risk of improper motor
        // config.
        robotHardware = new RobotHardwareGroup(hardwareMap);
        driverGamepad = GamepadWrapper.newDriverGamepad(gamepad1);
        operatorGamepad = GamepadWrapper.newOperatorGamepad(gamepad2);

        oldDrivetrain = new OldDrivetrain();
        oldIntake = new OldIntake();
        oldShooter = new OldShooter();
        oldVision = new OldVision();
        oldTurret = new OldTurret();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", "Square = Red, X = Blue");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        driverGamepad.update();
        if (driverGamepad.btnX.isHeld())
            isRed = true;
        if (driverGamepad.btnA.isHeld())
            isRed = false;

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
        // - The "::" means don't call this function, just pass it forward as a
        // parameter
        driverGamepad.leftBumper.whenHeld(() -> oldDrivetrain.setSpeedMultiplier(1));
        driverGamepad.leftBumper.whenNotHeld(() -> oldDrivetrain.setSpeedMultiplier(0.1));
        operatorGamepad.dpadUp.onPress(oldShooter::speedUpFlywheel);
        operatorGamepad.dpadDown.onPress(oldShooter::slowDownFlywheel);
        operatorGamepad.btnY.onPress(oldDrivetrain::resetIMU);
        operatorGamepad.btnB.onPress(oldTurret::resetTurretEncoder);
        operatorGamepad.leftBumper.whenHeld(oldIntake::eat);
        operatorGamepad.btnA.whenHeld(oldIntake::invert);
        operatorGamepad.rightTrigger.whenHeld(oldShooter::feed);
        operatorGamepad.btnA.whenHeld(oldShooter::reverseFeed);
        operatorGamepad.btnX.onPress(oldShooter::toggleFlywheel);
        driverGamepad.btnY.onPress(() -> isFieldDriving = !isFieldDriving);
        operatorGamepad.leftBumper.onPress(() -> useFlywheelLookups = !useFlywheelLookups);
    }

    @Override
    public void loop() {
        driverGamepad.update();
        operatorGamepad.update();

        // TODO: replace with follower.setTeleOpDrive() once Pedro Pathing is added
        oldDrivetrain.drive(
                new Vector2D(
                        -driverGamepad.raw.left_stick_y,
                        driverGamepad.raw.left_stick_x),
                driverGamepad.raw.right_stick_x,
                isFieldDriving);

        // oldTurret: right stick x for manual override, or auto-track the goal tag
        double stickX = operatorGamepad.raw.right_stick_x;
        if (Math.abs(stickX) > STICK_DEADBAND) {
            if (!turretManualMode) {
                turretManualMode = true;
            }
        } else {
            turretManualMode = false;
        }

        AprilTagDetection goalTag = oldVision.getTagById(isRed ? RED_GOAL : BLUE_GOAL);

        if (turretManualMode) {
            lastGoalTagTime = getRuntime();
            manualTurretAngle = oldTurret.getCurrentAngle();
            manualTurretAngle += stickX * 3;
            oldTurret.goTo(manualTurretAngle);
        } else {
            if (goalTag != null) {
                lastGoalTagTime = getRuntime();
                double bearing = Math.toDegrees(goalTag.ftcPose.bearing);
                double angle = Ballistics.calculateTurretAngle(bearing, oldTurret.getCurrentAngle());
                oldTurret.goTo(angle);
            } else if (getRuntime() - lastGoalTagTime > OldTurret.LOST_TAG_RETURN_DELAY) {
                oldTurret.returnHome();
            }
        }

        if (useFlywheelLookups && goalTag != null) {
            double distance = goalTag.ftcPose.range;
            OldShooter.setDesiredFlywheelRPS(Ballistics.calculateFlywheelRPS(distance));
        }

        oldIntake.update();
        oldShooter.update();
        oldTurret.update();

        // Telemetry
        simpleTelemetry.addData("OldIntake power", oldIntake.getPower());
        simpleTelemetry.addData("Feeder power", oldShooter.getFeederPower());
        simpleTelemetry.addData("Flywheel rps", oldShooter.getFlywheelRPS());
        simpleTelemetry.addData("Flywheel error", oldShooter.getFlywheelErrorRPS());
        simpleTelemetry.addData("OldDrivetrain speed", oldDrivetrain.getSpeedMultiplier());
        simpleTelemetry.addData("OldTurret angle", oldTurret.getCurrentAngle());
        simpleTelemetry.addData("OldTurret error", oldTurret.getErrorAngle());
        simpleTelemetry.addData("OldTurret mode", turretManualMode ? "MANUAL" : "AUTO");
        simpleTelemetry.addData("Field Centric", isFieldDriving);
        simpleTelemetry.addData("Use lookups", useFlywheelLookups);

        if (goalTag != null) {
            // Locale.US means use the 12.22 every time instead of 12,22 because some
            // countries use
            // the "," instead of the "." why I do not know.
            simpleTelemetry.addData(
                    "Tag distance", String.format(Locale.US, "%.2f m", goalTag.ftcPose.range));
            simpleTelemetry.addData(
                    "Tag bearing",
                    String.format(Locale.US, "%.1f deg", Math.toDegrees(goalTag.ftcPose.bearing)));
        } else {
            simpleTelemetry.addData("Tag distance", "no tag");
        }

        // panels graph feed
        simpleTelemetry.addData("flywheelRPS", oldShooter.getFlywheelRPS());
        simpleTelemetry.addData("flywheelTarget", OldShooter.desiredFlywheelRPS);
        simpleTelemetry.addData("flywheelError", oldShooter.getFlywheelErrorRPS());
        simpleTelemetry.addData("feederPower", oldShooter.getFeederPower());
        simpleTelemetry.addData("intakePower", oldIntake.getPower());

        simpleTelemetry.update();
        simpleTelemetry.update(telemetry);
    }
}
