package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.utils.AprilTags.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.utils.AprilTags.RED_GOAL;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.configs.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Ballistics;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utils.GamepadEx;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends OpMode {

    private final RobotHardware hw = new RobotHardware();

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private Vision vision;

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final GraphManager panelsGraph = PanelsGraph.INSTANCE.getManager();
    private final GamepadEx driverButtons = new GamepadEx();
    private final GamepadEx operatorButtons = new GamepadEx();

    private final GamepadManager driverPanelsGamepad = PanelsGamepad.INSTANCE.getFirstManager();
    private final GamepadManager operatorPanelsGamepad = PanelsGamepad.INSTANCE.getSecondManager();

    private boolean isFieldDriving = false;
    public static boolean isRed = false;

    private static final double TRIGGER_THRESHOLD = 0.5;
    private static final double STICK_DEADBAND = 0.15;
    private boolean turretManualMode = false;
    private double manualTurretAngle = 0;
    public static boolean useFlywheelLookups = true;

    @Override
    public void init() {
        hw.init(hardwareMap);

        drivetrain = new Drivetrain(hw);
        intake = new Intake(hw);
        shooter = new Shooter(hw);
        vision = new Vision(hw);
        turret = new Turret(hw);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", "Square = Red, X = Blue");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        Gamepad driverGamepad = getDriverGamepad();

        if (driverGamepad.x) {
            isRed = true;
        } else if (driverGamepad.a) {
            isRed = false;
        }
        telemetry.addData("Team", isRed ? "RED" : "BLUE");
        telemetry.addData("Switch", "Square = Red, X = Blue");
        telemetry.update();
    }

    @Override
    public void start() {}
    @Override
    public void loop() {
        Gamepad driverGamepad = getDriverGamepad();
        Gamepad operatorGamepad = getOperatorGamepad();

        driverButtons.update(driverGamepad);
        operatorButtons.update(operatorGamepad);

        // gamepad 1
        drivetrain.setSpeedMultiplier(driverButtons.lb.isHeld(), driverButtons.rb.isHeld());
        // TODO: replace with follower.setTeleOpDrive() once pedro is added
        drivetrain.drive(
                -driverGamepad.left_stick_y,
                driverGamepad.left_stick_x,
                driverGamepad.right_stick_x,
                isFieldDriving
        );

        if (driverButtons.y.wasPressed()) {
            isFieldDriving = !isFieldDriving;
        }

        // gamepad 2
        if (operatorButtons.dpadUp.wasPressed()) {shooter.speedUpFlywheel();}
        if (operatorButtons.dpadDown.wasPressed()) {shooter.slowDownFlywheel();}

        if (operatorButtons.y.wasPressed()) {drivetrain.resetIMU();}
        if (operatorButtons.b.wasPressed()) {turret.resetTurretEncoder();}

        if (operatorButtons.lt >= TRIGGER_THRESHOLD) {intake.eat();}
        if (operatorButtons.a.isHeld()) { intake.invert();}

        if (operatorGamepad.right_trigger >= TRIGGER_THRESHOLD){
            if (operatorButtons.a.isHeld()) {
                shooter.reverseFeed();
            } else {
                shooter.feed();
            }
        }
        if (gp2.x.wasPressed()) { shooter.toggleFlywheel();}
        if (gp2.lb.wasPressed()) { useFlywheelLookups = !useFlywheelLookups; }

        // turret: right stick x for manual override, or auto-track the goal tag
        double stickX = gamepad2.right_stick_x;
        if (Math.abs(stickX) > STICK_DEADBAND) {
            if (!turretManualMode) {
                turretManualMode = true;
            }
        } else {
            turretManualMode = false;
        }

        AprilTagDetection goalTag = vision.getTagById(isRed ? RED_GOAL : BLUE_GOAL);

        if (turretManualMode) {
            manualTurretAngle = turret.getCurrentAngle();
            manualTurretAngle += stickX * 3;
            turret.goTo(manualTurretAngle);
        } else {
            if (goalTag != null) {
                double bearing = Math.toDegrees(goalTag.ftcPose.bearing);
                double angle = Ballistics.calculateTurretAngle(bearing, turret.getCurrentAngle());
                turret.goTo(angle);
            }
        }

        if (useFlywheelLookups && goalTag != null) {
            double distance = goalTag.ftcPose.range;
            shooter.setDesiredFlywheelRPS(Ballistics.calculateFlywheelRPS(distance));

        }

        intake.update();
        shooter.update();
        turret.update();

        // Telemetry
        panelsTelemetry.addData("Intake power",  intake.getPower());
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
            panelsTelemetry.addData("Tag distance", String.format("%.2f m", goalTag.ftcPose.range));
            panelsTelemetry.addData("Tag bearing", String.format("%.1f deg", Math.toDegrees(goalTag.ftcPose.bearing)));
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

    private Gamepad getDriverGamepad() {
        return driverPanelsGamepad.asCombinedFTCGamepad(gamepad1);
    }

    private Gamepad getOperatorGamepad() {
        return operatorPanelsGamepad.asCombinedFTCGamepad(gamepad2);
    }
}
