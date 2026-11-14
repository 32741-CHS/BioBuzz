package org.firstinspires.ftc.teamcode.configs;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

@Configurable
public class Globals {
    // Remember Final = no change; static = same var shared across instances, so
    // update one
    // updates all;
    // REMOVE FINAL TO EDIT VIA PANELS


    /// Motor Tick Rates
    public static final double GOBILDA_5203_6000RPM = 28, GOBILDA_5203_312RPM = 537.7;
    /// Drive Train
    public static final double WHEEL_DIAMETER = 10.4;
    public static final double STICK_DEADBAND = 0.05;
    public static final int BLUE_GOAL = 20;
    public static final int RED_GOAL = 24;
    public static final int MAX_FPS = 15;
    /// Shooter
    public static final double
            FLYWHEEL_ERROR_TOL = 1.0,
            TRIGGER_THRESHOLD = 0.5,
            flywheelKP = 16.5,
            flywheelKF = 13.5;
    /// FEEDER
    public static final double FEEDER_POWER = 0.7;
    // Panels
    public static final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    public static final GraphManager panelsGraph = PanelsGraph.INSTANCE.getManager();
    /// Flywheel constants
    public static final double Gravity = 9.81,
            DecaySpin = 0.223, // Guessed, tweak it ig
            AerodynamicDrag = 0.00005320098663,
            AirPressure = 1.225,
            SlipagePercentage = 0.1,
            MagnusEffect = 0.019,
            TimeInterval = 0.01,
            SimulationTimeout = 2.5,
            RotationalDirection = -1,
            FlywheelDiameter = 0.096,
            BallGeneralMass = 0.03, // should change for nectar / polle,
            PollenMass = 0.0249,
            NectarMass = 0.0413,
            ShootingAngle = 65,
            InitialCamHeight = 0.37;
    // 22mm inward = pollen
    // 0mm inward = nectar
    // Feeder
    public static final NectarColorEnum NectarColor = NectarColorEnum.Red;
    public static Boolean isRedAlliance = true;
    public static boolean fieldCentricDriving = false;
    /// Servo
    public static double SERVO_BIG_BALL = 0.9;
    public static double SERVO_SMALL_BALL = 0.7;


    // Ball colors & Detection
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
