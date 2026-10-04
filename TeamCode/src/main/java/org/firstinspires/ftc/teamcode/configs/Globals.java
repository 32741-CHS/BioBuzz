package org.firstinspires.ftc.teamcode.configs;

public class Globals {
    //    Remember Final = no change; static = same var shared across instances, so update one
    // updates all;

    //    PID Constants
    public static final double K_PROPORTIONAL = 0; // Error magnifier
    public static final double K_INTEGRAL = 0; // Anti consistent Error
    public static final double K_DERIVATIVE = 0; // Anti oscillations

    //    The Robot Hardware object thing
    public static final RobotHardwareGroup robotHardwareGroup =
            new RobotHardwareGroup(); // Not Init yet, just exists.

    // Motor Tick Rates
    public static final double GOBILDA_5203_6000RPM = 28;
    public static final double GOBILDA_5203_312RPM = 537.7;

    // Drive Train
    public static final double WHEEL_DIAMETER = 10.4;
    public static final double STICK_DEADBAND = 0.05;
    public static final int BLUE_GOAL = 20;
    public static final int RED_GOAL = 24;
    public static final int MAX_FPS = 15;

    // Shooter
    public static final double FLYWHEEL_ERROR_TOL = 1.0;
    public static final double TRIGGER_THRESHOLD = 0.5;
    // Current Position
    public static double distanceToCamera;
    public static double cameraPositionX;
    public static double expectedRobotPositionX;
    // Vision
    public static double cameraPositionY;
    public static double expectedRobotPositionY;
    public static double realRobotPositionX;
    public static double realRobotPositionY;
    public static int currentGoal;
}
