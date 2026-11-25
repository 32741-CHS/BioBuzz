package org.firstinspires.ftc.teamcode.configs;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.utils.SimpleTelemetry;

@Configurable
public class Globals {
    // Remember Final = no change; static = same var shared across instances, so
    // update one
    // updates all;
    // REMOVE FINAL TO EDIT VIA PANELS


    /// Motor Tick Rates
    public static final double
            GOBILDA_5203_6000RPM = 28,
            GOBILDA_5203_312RPM = 537.7;


    /// Drive Train
    public static final double
            WHEEL_DIAMETER = 10.4,
            STICK_DEADBAND = 0.05;
    public static final int
            BLUE_GOAL = 20,
            RED_GOAL = 24,
            MAX_FPS = 15;
    /// Shooter
    public static final double
            FLYWHEEL_ERROR_TOL = 1.0,
            TRIGGER_THRESHOLD = 0.5,
            flywheelKP = 16.5,
            flywheelKF = 13.5;
    /// FEEDER
    public static final double FEEDER_POWER = 0.7;
    // Panels
    public static final SimpleTelemetry simpleTelemetry = new SimpleTelemetry();
    public static final TelemetryManager panelsGraph = PanelsTelemetry.INSTANCE.getTelemetry();
    /// Flywheel constants
    public static final double Gravity = 9.81,
            DECAY_SPIN = 0.223, // Guessed, tweak it ig
            AERODYNAMIC_DRAG = 0.00005320098663,
            AIR_PRESSURE = 1.225,
            SLIPAGE_PERCENTAGE = 0.1,
            MAGNUS_EFFECT = 0.019,
            TIME_INTERVAL = 0.01,
            SIMULATION_TIMEOUT = 2.5,
            ROTATIONAL_DIRECTION = -1,
            FLYWHEEL_DIAMETER = 0.096,
            BALL_GENERAL_MASS = 0.03, // should change for nectar / polle,
            POLLEN_MASS = 0.0249,
            NECTAR_MASS = 0.0413,
            SHOOTING_ANGLE = 65,
            INITIAL_CAM_HEIGHT = 0.37;
    // 22mm inward = pollen
    // 0mm inward = nectar
    // Feeder
    public static final NectarColorEnum NectarColor = NectarColorEnum.Red;
    public static boolean fieldCentricDriving = false;
    /// Light Op Mode
    public static double FLYWHEEL_POWER = 0.588;

    public static boolean isRedAlliance = true;

    /// Servo Ranges
    public static double
            SERVO_BIG_BALL = 0.9,
            SERVO_SMALL_BALL = 0.7;


    /// Ball colors & Detection
    public enum NectarColorEnum {
        Red,
        Blue
    }

    public enum BallType {
        Big,
        Small,
        None
    }

}
