package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.Globals.AERODYNAMIC_DRAG;
import static org.firstinspires.ftc.teamcode.configs.Globals.BALL_GENERAL_MASS;
import static org.firstinspires.ftc.teamcode.configs.Globals.DECAY_SPIN;
import static org.firstinspires.ftc.teamcode.configs.Globals.FLYWHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.configs.Globals.FLYWHEEL_ERROR_TOL;
import static org.firstinspires.ftc.teamcode.configs.Globals.GOBILDA_5203_6000RPM;
import static org.firstinspires.ftc.teamcode.configs.Globals.Gravity;
import static org.firstinspires.ftc.teamcode.configs.Globals.INITIAL_CAM_HEIGHT;
import static org.firstinspires.ftc.teamcode.configs.Globals.ROTATIONAL_DIRECTION;
import static org.firstinspires.ftc.teamcode.configs.Globals.SHOOTING_ANGLE;
import static org.firstinspires.ftc.teamcode.configs.Globals.SIMULATION_TIMEOUT;
import static org.firstinspires.ftc.teamcode.configs.Globals.SLIPAGE_PERCENTAGE;
import static org.firstinspires.ftc.teamcode.configs.Globals.TIME_INTERVAL;
import static org.firstinspires.ftc.teamcode.configs.Globals.panelsGraph;
import static org.firstinspires.ftc.teamcode.configs.Globals.simpleTelemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.configs.BotHardware;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

@Configurable
public class Flywheel implements SubsystemBase {
    private static final double CameraDisplacementX = 0.06;
    private static final double CameraHeight = 0.37;
    private static final double HeightFromAprilTag = 0.15; // tweak if not good
    // NEW: lumped Magnus (spin lift) constant, units 1/m. 0 = Magnus off. Tune on the field.
    private static final double MagnusCoefficient = 0.019;
    // config
    public static double desiredFlywheelRPS = 13;
    public static double maximumFlywheelRPM = 6000;
    public static double minimumFlywheelRPM = 0;
    public static boolean useDynamicSpeed = true;
    public static boolean canSpinFlywheel = true;
    public static boolean doSimulation = true; // set to false if lag
    // private final ColourDetector colourDetector;
    private DcMotorEx flywheel;
    // private WebcamName insideCam;
    // goal stuff
    private double distance = Double.NaN;
    private double zRadians = Double.NaN;

    public Flywheel(/*, ColourDetector cam*/) {
        // this.colourDetector = cam;
    }

    public static void setDesiredFlywheelRPS(double rps) {
        desiredFlywheelRPS = rps;
    }

    public void changeGoal(double distance, double zRadians) {
        this.distance = distance;
        this.zRadians = zRadians;
    }

    @Override
    public void init(BotHardware botHardware) {
        flywheel = (DcMotorEx) botHardware.flywheel;

        // this.insideCam = botHardware.insideCam;
    }

    @Override
    public void update() {
        if (useDynamicSpeed) UpdateDynamicRPS();

        flywheel.setVelocity(canSpinFlywheel ? desiredFlywheelRPS * GOBILDA_5203_6000RPM : 0);

        panelsGraph.addData("Flywheel Vel", canSpinFlywheel ? desiredFlywheelRPS * GOBILDA_5203_6000RPM : 0);
        panelsGraph.addData("Flywheel RPS", getFlywheelRPS());
        panelsGraph.addData("Flywheel Error", getFlywheelErrorRPS());
    }

    private void UpdateDynamicRPS() {
        if (Double.isNaN(distance) || Double.isNaN(zRadians)) return;

        double horizontalDistance = CameraDisplacementX + distance * Math.cos(zRadians);
        double verticalDistance = distance * Math.sin(zRadians) + CameraHeight;

        // MATH FIX 1: standard projectile equation, ball speed needed with NO air resistance
        // v^2 = g x^2 / (2 cos^2(theta) (x tan(theta) - dy))
        double heightDiff = verticalDistance - INITIAL_CAM_HEIGHT;
        double angleRad = Math.toRadians(SHOOTING_ANGLE);
        double denominator = 2 * Math.pow(Math.cos(angleRad), 2)
                * (horizontalDistance * Math.tan(angleRad) - heightDiff);
        if (denominator <= 0)
            return; // target can't be hit at this launch angle (only added line that isn't maths)
        double velocityInVacuum = Math.sqrt(Gravity * horizontalDistance * horizontalDistance / denominator);

        double flywheelRadius = FLYWHEEL_DIAMETER / 2;

        // MATH FIX 2: ball leaves slower than the wheel surface, so DIVIDE by (1 - slip).
        // MATH FIX 3: RPS = surface speed / circumference (no *60 factors).
        double desiredRPS = (velocityInVacuum / (1 - SLIPAGE_PERCENTAGE)) / (Math.PI * FLYWHEEL_DIAMETER);

        // MATH FIX 4: simulation starts from the speed we just calculated, not the old desiredFlywheelRPS
        double initialVelocityX = Math.cos(angleRad) * velocityInVacuum;
        double initialVelocityY = Math.sin(angleRad) * velocityInVacuum;

        double Spin, Geff, BallRPM, Vx, Vy, Px, Py;
        BallRPM = desiredRPS * 60; // MATH FIX 5: spin comes from the wheel speed we are asking for
        Vx = initialVelocityX;
        Vy = initialVelocityY;
        Px = 0; // MATH FIX 6: horizontalDistance is already measured from the launch point
        Py = INITIAL_CAM_HEIGHT;

        double ballMass = BALL_GENERAL_MASS;
        /*
        if (colourDetector.getBallType(insideCam) == Globals.BallType.Big) {
            ballMass = NectarMass;
        } else if (colourDetector.getBallType(insideCam) == Globals.BallType.Small) {
            ballMass = PollenMass;
        }
        */

        // AerodynamicDrag is now k in a = -k|v|v (units 1/m, = 0.5*rho*Cd*A/mass), scaled for ball mass
        double dragK = AERODYNAMIC_DRAG * BALL_GENERAL_MASS / ballMass;


        if (doSimulation) {
            double currTime = 0;
            boolean simulationComplete = false;
            while (!simulationComplete) {
                double speed = Math.hypot(Vx, Vy);
                Spin = (2 * Math.PI) / (60) * BallRPM * flywheelRadius;
                // MATH FIX 7: Magnus lift grows WITH spin and speed (was divided by spin). Vertical-only approximation.
                // RotationalDirection: -1 = backspin (lifts the ball), +1 = topspin.
                Geff = Gravity + ROTATIONAL_DIRECTION * MagnusCoefficient * Spin * speed;
                // MATH FIX 8: exponential decay per step (was multiplying by currTime, which zeroed it)
                BallRPM = BallRPM * Math.exp(-DECAY_SPIN * TIME_INTERVAL);
                // MATH FIX 9: drag is proportional to speed^2, and gravity acts at full strength on Vy
                Vx = Vx - dragK * speed * Vx * TIME_INTERVAL;
                Vy = Vy - (Geff + dragK * speed * Vy) * TIME_INTERVAL;
                Px = Px + Vx * TIME_INTERVAL;
                Py = Py + Vy * TIME_INTERVAL;

                if (Py <= 0.001) {
                    simpleTelemetry.addData("Flywheel Simulation", "Ball altitude cannot be less than zero.");
                    desiredRPS += desiredRPS / 100 * 5;
                    break;
                }

                // MATH FIX 10: "reached" now means crossing the goal distance AT or ABOVE the goal height
                if (Px >= horizontalDistance) {
                    if (Py >= verticalDistance) {
                        simpleTelemetry.addData("Flywheel Simulation", "Ball reached simulated position.");
                    } else {
                        simpleTelemetry.addData("Flywheel Simulation", "Ball arrived below target, desiredRPS increased by 5%.");
                        desiredRPS += desiredRPS / 100 * 5;
                    }
                    simulationComplete = true;
                    break;
                }

                if (currTime > SIMULATION_TIMEOUT) {
                    simulationComplete = true;
                    if (!(Px >= horizontalDistance)) {
                        simpleTelemetry.addData("Flywheel Simulation", "Ball failed to reach simulated position.");
                        simpleTelemetry.addData("Flywheel Simulation", "desiredRPS increased by 5% to account for failure.");
                        desiredRPS += desiredRPS / 100 * 5;
                    }
                    break;
                }

                currTime += TIME_INTERVAL;
            }

        }

        desiredFlywheelRPS = desiredRPS;
    }

    public double getFlywheelRPS() {
        return flywheel.getVelocity() / GOBILDA_5203_6000RPM;
    }

    public double getFlywheelErrorRPS() {
        return getFlywheelRPS() - desiredFlywheelRPS;
    }

    public void runFlywheel() {

    }

    public boolean isReadyForFeed() {
        return Math.abs(getFlywheelErrorRPS()) <= FLYWHEEL_ERROR_TOL;
    }

}