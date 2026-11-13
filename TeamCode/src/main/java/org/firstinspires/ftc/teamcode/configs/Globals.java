package org.firstinspires.ftc.teamcode.configs;

// ONLY CONSTANTS!! Avoid NonConstants as they are a bit dangerous as we need to wipe them before every match


import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

public class Globals {
    // Remember Final = no change; static = same var shared across instances, so
    // update one
    // updates all;

    // PID Constants
    public static final double K_PROPORTIONAL = 0; // Error magnifier
    public static final double K_INTEGRAL = 0; // Anti consistent Error
    public static final double K_DERIVATIVE = 0; // Anti oscillations
    // Motor Tick Rates
    public static final double GOBILDA_5203_6000RPM = 28;
    public static final double GOBILDA_5203_312RPM = 537.7;

    // Drive Train
    public static final double WHEEL_DIAMETER = 10.4;
    public static final double STICK_DEADBAND = 0.05;
    public static final int BLUE_GOAL = 20;
    public static final int RED_GOAL = 24;
    public static final int MAX_FPS = 15;

    // OldShooter
    public static final double
            FLYWHEEL_ERROR_TOL = 1.0,
            TRIGGER_THRESHOLD = 0.5,
            flywheelKP = 16.5,
            flywheelKF = 13.5;
    public static final NectarColorEnum NectarColor = NectarColorEnum.Red;
    // FEEDER
    public static final double FEEDER_POWER = 0.7;
    // Panels
    public static final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    public static final GraphManager panelsGraph = PanelsGraph.INSTANCE.getManager();
    public static boolean fieldCentricDriving = false;

    // Ball colors & Detection
    public enum NectarColorEnum {
        Red,
        Blue
    }

    // Flywheel constants
    public static final double Gravity = 9.81;
    private static final double DragCoefficient = 0.4;
    public static final double DecaySpin = 0.223; // Guessed, tweak it ig
    public static final double AerodynamicDrag = 0.00005320098663;
    public static final double AirPressure = 1.225;
    public static final double SlipagePercentage = 0.1;
    public static final double MagnusEffect = 0.019;
    public static final double TimeInterval = 0.01;
    public static final double SimulationTimeout = 2.5;
    public static final double RotationalDirection = -1;
    public static final double FlywheelDiameter = 0.096;
    public static final double BallGeneralMass = 0.03; // should change for nectar / pollen
    public static final double PollenMass = 0.0249;
    public static final double NectarMass = 0.0413;
    public static final double ShootingAngle = 65;
    public static final double InitialCamHeight = 0.37;

    // Feeder

    // 22mm inward = pollen
    // 0mm inward = nectar
    public static final double PollenInwardAmount = 22;
    public static final double NectarInwardAmount = 0;

}
