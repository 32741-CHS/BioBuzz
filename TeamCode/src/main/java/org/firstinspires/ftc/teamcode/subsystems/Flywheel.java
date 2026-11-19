package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.Globals.panelsGraph;
import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.configs.Globals;
import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.BallType;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

/**
 * FTC RAMO flywheel subsystem.
 *
 * This is a drop-in replacement for the Flywheel.java in BioBuzz-main.
 * It keeps the existing constructor, changeGoal(), RPS getters, runFlywheel(),
 * and isReadyForFeed() interfaces used by MainTeleOp, GoalTracker and Feeder.
 *
 * Units:
 * - FTC AprilTag range received through changeGoal(): inches, horizontal X-Y range.
 * - FTC bearing/elevation arguments: radians (GoalTracker already converts them).
 * - Internally: metres, kilograms, seconds, radians, and rad/s.
 * - Flywheel encoder: ticks/s, using Globals.GOBILDA_5203_6000RPM ticks/rev.
 *
 * IMPORTANT: The solver is a physics model and starting point. It cannot promise
 * a perfect shot. Measure the robot-specific inertia, spin ratio, sensor offset,
 * hood angle, ball exit height and energy transfer on the real robot.
 */
@Configurable
public class Flywheel implements SubsystemBase {

    // Existing public controls
    public static double desiredFlywheelRPS = 13.0;
    public static double maximumFlywheelRPM = 6000.0;
    public static double minimumFlywheelRPM = 0.0;
    public static boolean useDynamicSpeed = true;
    public static boolean canSpinFlywheel = false;

    // Distances are metres. Angles are degrees unless the name says "Rad".
    public static double CAMERA_HEIGHT_M = 0.37;

    public static double CAMERA_AHEAD_OF_SHOOTER_M = 0.06;

    /**
     * Reference-target vertical offset, applied to target height.
     * Positive means the actual point to hit is above the AprilTag centre.
     * Verify this sign and value against the actual target/tag mounting.
     */
    public static double TARGET_POINT_ABOVE_TAG_CENTRE_M = 0.15;

    /**
     * Pollen diameter 2.8 in and Nectar diameter 3.6 in, from the BIOBUZZ
     * competition manual. These may be adjusted for measured ball variation.
     */
    public static double POLLEN_RADIUS_M = 0.03556;
    public static double NECTAR_RADIUS_M = 0.04572;

    public static double BALL_SPIN_RATIO = 1.1;

    public static double BALL_INERTIA_BETA = 2.0 / 3.0;

    /** Fraction of wheel energy transferred to the ball; initial estimate only. */
    public static double ENERGY_TRANSFER_EFFICIENCY = 0.70;

    /** Motor revolutions per flywheel revolution; default preserves old 1:1 code. */
    public static double MOTOR_REVOLUTIONS_PER_FLYWHEEL_REV = 1.0;

    public static double FLYWHEEL_INERTIA_KG_M2 = 0.00025;

    /** Solver configuration. */
    public static int MAX_COEFFICIENT_ITERATIONS = 20;
    public static double COEFFICIENT_SPEED_TOLERANCE_MPS = 0.001;
    public static double TARGET_STALE_SECONDS = 0.35;

    // Existing Globals.flywheelKP and Globals.flywheelKF are deliberately used.
    // These I/D terms are added here because Globals currently defines P and F
    // only. Start at zero; increase only after testing on the real mechanism.
    public static double SHOOTER_KI = 0.0;
    public static double SHOOTER_KD = 0.0;

    // Pollen CFD table. Spin labels are angular velocity in rad/s, NOT RPM.
    // Cd and Cl are means of the final 20 rows from the supplied CSV files.
    // The 75 point is interpolated because that CSV contains Ux,Uy,Uz,p only.
    // The 150 point assumes the ambiguous "1,50" filename means 150 rad/s.
    private static final double[][] POLLEN_CFD = new double[][] {
            // spin rad/s,       Cd,        Cl
            {   0.0,          0.586302,  0.101643 },
            {  75.0,          0.498444,  0.384987 },
            { 150.0,          0.410585,  0.668330 },
            { 270.0,          0.982075,  0.869447 },
            { 420.0,          1.103894,  1.072725 },
            { 600.0,          0.429050,  1.537901 },
            {1500.0,          1.413068,  2.239814 }
    };

    // it uses the Pollen curve provisionally; its larger radius and different
    // measured mass are still used in the physical calculation below.

    private static final double INCH_TO_METRE = 0.0254;
    private static final double TWO_PI = 2.0 * Math.PI;

    // Existing hardware/subsystem dependencies.
    private DcMotorEx flywheel;
    private Drivetrain drivetrain;

    private double distanceInput = Double.NaN;
    private double bearingRadians = 0.0;
    private double elevationRadians = Double.NaN;
    private boolean inputIsDirectSlantRange = false;
    private long lastGoalUpdateNanos = 0L;

    private BallType currentBallType = BallType.Big;
    private boolean hasValidDynamicTarget = false;
    private ShotSolution lastSolution;

    private double lastAppliedP = Double.NaN;
    private double lastAppliedI = Double.NaN;
    private double lastAppliedD = Double.NaN;
    private double lastAppliedF = Double.NaN;

    public Flywheel(Drivetrain dt, ColourDetector colourDetector) {
        this.drivetrain = dt;
        this.currentBallType = BallType.Big;
    }

    /** Select the ball manually. BallType.None is ignored. */
    public void setBallType(BallType ballType) {
        if (ballType != null && ballType != BallType.None) {
            currentBallType = ballType;
        }
    }

    /** Returns the currently selected ball type. */
    public BallType getBallType() {
        return currentBallType;
    }

    public void changeGoal(double distance, double zRadians) {
        changeGoal(distance, 0.0, zRadians);
    }

    /**
     * Bearing-aware version. distance is FTC ftcPose.range in inches; bearing
     * and elevation are radians. Recommended GoalTracker call:
     * flywheel.changeGoal(distanceToGoal, xRadians, zRadians);
     */
    public void changeGoal(double distance, double bearingRad, double elevationRad) {
        this.distanceInput = distance;
        this.bearingRadians = bearingRad;
        this.elevationRadians = elevationRad;
        this.inputIsDirectSlantRange = false;
        this.lastGoalUpdateNanos = System.nanoTime();
    }

    /**
     * Optional direct-range API for callers that already have true 3D slant
     * range in metres. The existing GoalTracker does not call this overload.
     */
    public void changeGoalFromSlantRange(double directSlantRangeM,
                                         double bearingRad,
                                         double elevationRad) {
        this.distanceInput = directSlantRangeM;
        this.bearingRadians = bearingRad;
        this.elevationRadians = elevationRad;
        this.inputIsDirectSlantRange = true;
        this.lastGoalUpdateNanos = System.nanoTime();
    }

    private void updateDynamicRPS() {
        if (!useDynamicSpeed) {
            hasValidDynamicTarget = true;
            return;
        }

        if (lastGoalUpdateNanos == 0L
                || (System.nanoTime() - lastGoalUpdateNanos) * 1.0e-9 > TARGET_STALE_SECONDS) {
            hasValidDynamicTarget = false;
            panelsTelemetry.addData("[Flywheel] Dynamic target", "Waiting for a fresh target measurement");
            return;
        }

        if (!finite(distanceInput) || distanceInput <= 0.0 || !finite(elevationRadians)
                || !finite(bearingRadians)) {
            hasValidDynamicTarget = false;
            panelsTelemetry.addData("[Flywheel] Solver", "Invalid range/bearing/elevation input");
            return;
        }

        lastSolution = solveShot(distanceInput, bearingRadians, elevationRadians,
                inputIsDirectSlantRange, currentBallType);

        panelsTelemetry.addData("[Flywheel] Ball type", currentBallType.name());
        panelsTelemetry.addData("[Flywheel] Range input", distanceInput);
        panelsTelemetry.addData("[Flywheel] Range input unit", inputIsDirectSlantRange
                ? "metres, direct slant range" : "inches, FTC horizontal range");
        panelsTelemetry.addData("[Flywheel] Bearing deg", Math.toDegrees(bearingRadians));
        panelsTelemetry.addData("[Flywheel] Elevation deg", Math.toDegrees(elevationRadians));

        if (!lastSolution.valid) {
            hasValidDynamicTarget = false;
            panelsTelemetry.addData("[Flywheel] Solver status", "INVALID: " + lastSolution.error);
            return;
        }

        double rpmBeforeLimit = lastSolution.flywheelRPM;
        double rpmLimited = clamp(rpmBeforeLimit, minimumFlywheelRPM, maximumFlywheelRPM);
        desiredFlywheelRPS = rpmLimited / 60.0;
        hasValidDynamicTarget = true;

        // Geometry and ballistics telemetry.
        panelsTelemetry.addData("[Flywheel] Direct slant range m", lastSolution.slantRangeM);
        panelsTelemetry.addData("[Flywheel] Horizontal shot distance m", lastSolution.horizontalDistanceM);
        panelsTelemetry.addData("[Flywheel] Target height above shooter m", lastSolution.targetHeightM);
        panelsTelemetry.addData("[Flywheel] Launch angle deg", Math.toDegrees(lastSolution.launchAngleRad));
        panelsTelemetry.addData("[Flywheel] Required ball exit speed m/s", lastSolution.exitSpeedMps);
        panelsTelemetry.addData("[Flywheel] Calculated ball spin rad/s", lastSolution.ballSpinRadPerSec);
        panelsTelemetry.addData("[Flywheel] Required flywheel rad/s", lastSolution.flywheelRadPerSec);
        panelsTelemetry.addData("[Flywheel] Calculated flywheel RPM", rpmBeforeLimit);
        panelsTelemetry.addData("[Flywheel] Applied target RPM", rpmLimited);
        panelsTelemetry.addData("[Flywheel] RPM limit active", Math.abs(rpmBeforeLimit - rpmLimited) > 1.0e-6);

        // Aerodynamic lookup telemetry.
        panelsTelemetry.addData("[Flywheel] CFD Cd", lastSolution.cd);
        panelsTelemetry.addData("[Flywheel] CFD Cl", lastSolution.cl);
        panelsTelemetry.addData("[Flywheel] CFD lower spin rad/s", lastSolution.lowerSpinRadPerSec);
        panelsTelemetry.addData("[Flywheel] CFD upper spin rad/s", lastSolution.upperSpinRadPerSec);
        panelsTelemetry.addData("[Flywheel] CFD iterations", lastSolution.iterations);
        panelsTelemetry.addData("[Flywheel] CFD coefficient convergence", lastSolution.coefficientsConverged);
        panelsTelemetry.addData("[Flywheel] CFD 75 estimate used", lastSolution.estimatedPointUsed);
        panelsTelemetry.addData("[Flywheel] CFD unconverged run used", lastSolution.unconvergedPointUsed);
        panelsTelemetry.addData("[Flywheel] CFD table clamped", lastSolution.tableClamped);
        panelsTelemetry.addData("[Flywheel] Nectar Pollen fallback", lastSolution.nectarFallbackUsed);
        panelsTelemetry.addData("[Flywheel] CFD note", lastSolution.coefficientNote);

        if (Math.abs(rpmBeforeLimit - rpmLimited) > 1.0e-6) {
            panelsTelemetry.addData("[Flywheel] WARNING", "Calculated RPM exceeded configured motor limit");
        }
    }

    private ShotSolution solveShot(double inputDistance,
                                   double bearingRad,
                                   double elevationRad,
                                   boolean inputIsSlant,
                                   BallType ballType) {
        ShotSolution result = new ShotSolution();
        result.valid = false;

        if (Math.abs(elevationRad) >= Math.toRadians(85.0)) {
            result.error = "Elevation is too close to vertical";
            return result;
        }

        final double horizontalCameraRangeM;
        final double slantRangeM;
        final double bearingCos = Math.cos(bearingRad);
        final double elevationTan = Math.tan(elevationRad);

        // In the FTC SDK, bearing is atan2(-x, y), elevation is atan2(z, y),
        // and range is hypot(x, y). Therefore the vertical displacement is
        // y*tan(elevation), where y = horizontalRange*cos(bearing).
        // This reconstructs the true 3D camera-to-tag distance without
        // treating FTC's horizontal range as a slant range.
        if (inputIsSlant) {
            slantRangeM = inputDistance;
            horizontalCameraRangeM = slantRangeM
                    / Math.sqrt(1.0 + bearingCos * bearingCos * elevationTan * elevationTan);
        } else {
            horizontalCameraRangeM = inputDistance * INCH_TO_METRE;
            double forwardFromCameraM = horizontalCameraRangeM * bearingCos;
            double verticalFromCameraM = forwardFromCameraM * elevationTan;
            slantRangeM = Math.hypot(horizontalCameraRangeM, verticalFromCameraM);
        }

        if (!positive(horizontalCameraRangeM) || !positive(slantRangeM)) {
            result.error = "Range must be positive";
            return result;
        }

        // Reference-sensor target vector. Bearing accounts for the offset
        // between the target-measurement reference point and shooter exit.
        final double targetForwardFromCameraM = horizontalCameraRangeM * bearingCos;
        final double targetLeftFromCameraM = horizontalCameraRangeM * Math.sin(bearingRad);
        final double targetForwardFromShooterM = targetForwardFromCameraM
                + CAMERA_AHEAD_OF_SHOOTER_M;
        final double horizontalDistanceM = Math.hypot(targetForwardFromShooterM,
                targetLeftFromCameraM);

        // Reconstruct target height from the supplied forward-axis elevation.
        final double cameraToTagVerticalM = targetForwardFromCameraM * elevationTan;
        final double targetHeightAboveShooterM = cameraToTagVerticalM
                + CAMERA_HEIGHT_M - Globals.InitialCamHeight
                + TARGET_POINT_ABOVE_TAG_CENTRE_M;

        // Globals.ShootingAngle is stored as degrees in the supplied project.
        final double launchAngleRad = Math.toRadians(Globals.ShootingAngle);
        if (!positive(horizontalDistanceM)) {
            result.error = "Horizontal shot distance is invalid";
            return result;
        }
        if (!finite(targetHeightAboveShooterM)) {
            result.error = "Target height is invalid";
            return result;
        }
        if (!positive(Globals.AirPressure) || !positive(FLYWHEEL_INERTIA_KG_M2)
                || !positive(MOTOR_REVOLUTIONS_PER_FLYWHEEL_REV)
                || !(ENERGY_TRANSFER_EFFICIENCY > 0.0 && ENERGY_TRANSFER_EFFICIENCY <= 1.0)
                || !positive(BALL_SPIN_RATIO) || !positive(BALL_INERTIA_BETA)) {
            result.error = "Check air density, inertia, efficiency, spin ratio and beta settings";
            return result;
        }

        final boolean isPollen = ballType == BallType.Small;
        final double ballMassKg = isPollen ? Globals.PollenMass : Globals.NectarMass;
        final double ballRadiusM = isPollen ? POLLEN_RADIUS_M : NECTAR_RADIUS_M;
        final double flywheelRadiusM = Globals.FlywheelDiameter / 2.0;

        if (!positive(ballMassKg) || !positive(ballRadiusM) || !positive(flywheelRadiusM)) {
            result.error = "Ball mass/radius or flywheel radius is invalid";
            return result;
        }

        final double d = horizontalDistanceM;
        final double dh = targetHeightAboveShooterM;
        final double theta = launchAngleRad;
        final double D = d * Math.tan(theta) - dh;
        if (!(D > 1.0e-6)) {
            result.error = "Target is above the reachable launch line at the configured hood angle";
            return result;
        }
        if (Math.abs(Math.cos(theta)) < 1.0e-6) {
            result.error = "Launch angle is too close to vertical";
            return result;
        }

        double speedGuess = Math.sqrt(Globals.Gravity * d * d / (2.0 * D)) / Math.cos(theta);
        if (!positive(speedGuess)) {
            result.error = "Could not calculate initial ball exit speed";
            return result;
        }

        double exitSpeed = Double.NaN;
        double spinRadPerSec = Double.NaN;
        AeroResult aero = null;
        boolean converged = false;
        int iterations = 0;

        final int safeIterationLimit = Math.max(1, Math.min(64, MAX_COEFFICIENT_ITERATIONS));
        for (int i = 0; i < safeIterationLimit; i++) {
            iterations = i + 1;
            spinRadPerSec = BALL_SPIN_RATIO * speedGuess / ballRadiusM;
            aero = getCoefficients(ballType, spinRadPerSec);

            // Globals.RotationalDirection is -1 for backspin in the original
            // Flywheel code. That direction creates positive upward lift here.
            double signedCl = aero.cl * (Globals.RotationalDirection < 0.0 ? 1.0 : -1.0);
            double candidateSpeed = requiredExitSpeed(d, dh, theta,
                    ballMassKg, ballRadiusM, aero.cd, signedCl);
            if (!positive(candidateSpeed)) {
                result.error = "Trajectory equation failed; check launch angle, target height and CFD values";
                return result;
            }

            double error = Math.abs(candidateSpeed - speedGuess);
            exitSpeed = candidateSpeed;
            if (error <= COEFFICIENT_SPEED_TOLERANCE_MPS) {
                converged = true;
                speedGuess = candidateSpeed;
                break;
            }
            speedGuess = 0.5 * speedGuess + 0.5 * candidateSpeed;
        }

        // Re-evaluate so the reported Cd/Cl and spin match the final speed.
        spinRadPerSec = BALL_SPIN_RATIO * speedGuess / ballRadiusM;
        aero = getCoefficients(ballType, spinRadPerSec);
        double signedCl = aero.cl * (Globals.RotationalDirection < 0.0 ? 1.0 : -1.0);
        exitSpeed = requiredExitSpeed(d, dh, theta,
                ballMassKg, ballRadiusM, aero.cd, signedCl);
        if (!positive(exitSpeed)) {
            result.error = "Final trajectory evaluation failed";
            return result;
        }
        spinRadPerSec = BALL_SPIN_RATIO * exitSpeed / ballRadiusM;
        converged = converged || Math.abs(exitSpeed - speedGuess) <= COEFFICIENT_SPEED_TOLERANCE_MPS;

        // Supplied FlywheelRpm energy-transfer equation.
        final double S = BALL_SPIN_RATIO;
        final double wheelRadPerSec = exitSpeed * Math.sqrt(
                Math.pow((1.0 + S) / flywheelRadiusM, 2.0)
                        + ballMassKg * (1.0 + BALL_INERTIA_BETA * S * S)
                        / (ENERGY_TRANSFER_EFFICIENCY * FLYWHEEL_INERTIA_KG_M2));
        final double wheelRpm = wheelRadPerSec * 60.0 / TWO_PI;

        if (!positive(wheelRadPerSec) || !positive(wheelRpm)) {
            result.error = "Flywheel RPM equation returned an invalid value";
            return result;
        }

        result.valid = true;
        result.error = "";
        result.slantRangeM = slantRangeM;
        result.horizontalDistanceM = horizontalDistanceM;
        result.targetHeightM = targetHeightAboveShooterM;
        result.launchAngleRad = launchAngleRad;
        result.exitSpeedMps = exitSpeed;
        result.ballSpinRadPerSec = spinRadPerSec;
        result.flywheelRadPerSec = wheelRadPerSec;
        result.flywheelRPM = wheelRpm;
        result.cd = aero.cd;
        result.cl = aero.cl;
        result.lowerSpinRadPerSec = aero.lowerSpinRadPerSec;
        result.upperSpinRadPerSec = aero.upperSpinRadPerSec;
        result.iterations = iterations;
        result.coefficientsConverged = converged;
        result.estimatedPointUsed = aero.estimatedPointUsed;
        result.unconvergedPointUsed = aero.unconvergedPointUsed;
        result.tableClamped = aero.tableClamped;
        result.nectarFallbackUsed = !isPollen;
        result.coefficientNote = aero.note;
        return result;
    }

    /** Closed-form trajectory equation from the supplied FlywheelRpm.java. */
    private static double requiredExitSpeed(double d, double dh, double theta,
                                            double massKg, double radiusM,
                                            double cd, double cl) {
        final double D = d * Math.tan(theta) - dh;
        if (!(D > 1.0e-6)) return Double.NaN;

        final double area = Math.PI * radiusM * radiusM;
        final double kD = Globals.AirPressure * cd * area / (2.0 * massKg);
        final double kL = Globals.AirPressure * cl * area / (2.0 * massKg);

        final double p0 = Math.tan(theta);
        final double pd = 2.0 * dh / d - p0;
        final double a = 2.0 * D / (d * d);
        if (!finite(a) || Math.abs(a) < 1.0e-10) return Double.NaN;

        final double pathLengthTerm = (F(p0) - F(pd)) / (2.0 * a);
        if (!(pathLengthTerm > 0.0) || !finite(pathLengthTerm)) return Double.NaN;

        final double xi = 2.0 * kD * pathLengthTerm;
        if (!finite(xi) || xi > 50.0) return Double.NaN;
        final double dragAmplification = expm1MinusXOverX2(xi);
        if (!(dragAmplification > 0.0) || !finite(dragAmplification)) return Double.NaN;

        final double integralI = (Math.pow(1.0 + p0 * p0, 2.5)
                - Math.pow(1.0 + pd * pd, 2.5)) / 5.0
                - pd * (H(p0) - H(pd));
        final double magnusRise = kL / (a * a) * (5.0 / 3.0 * integralI
                - Math.pow(1.0 + p0 * p0, 1.5)
                * (p0 - pd) * (p0 - pd) / 3.0);

        final double denominator = 2.0 * (D + magnusRise);
        if (!(denominator > 1.0e-9) || !finite(denominator)) return Double.NaN;

        final double cosTheta = Math.cos(theta);
        final double speedSquared = Globals.Gravity * d * d * dragAmplification
                / denominator / (cosTheta * cosTheta);
        if (!(speedSquared > 0.0) || !finite(speedSquared)) return Double.NaN;
        return Math.sqrt(speedSquared);
    }

    /** Linear interpolation of Pollen CFD coefficients; Nectar uses same curve provisionally. */
    private static AeroResult getCoefficients(BallType ballType, double spinRadPerSec) {
        final double spin = Math.abs(spinRadPerSec);
        final double firstSpin = POLLEN_CFD[0][0];
        final double lastSpin = POLLEN_CFD[POLLEN_CFD.length - 1][0];

        if (!finite(spin)) {
            return new AeroResult(Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                    false, false, false, "Spin value is not finite");
        }

        for (int i = 0; i < POLLEN_CFD.length; i++) {
            if (Math.abs(spin - POLLEN_CFD[i][0]) < 1.0e-9) {
                return aeroAtPoint(i, spin, false, ballType);
            }
        }

        if (spin <= firstSpin) return aeroAtPoint(0, spin, spin < firstSpin, ballType);
        if (spin >= lastSpin) return aeroAtPoint(POLLEN_CFD.length - 1,
                spin, spin > lastSpin, ballType);

        for (int i = 0; i < POLLEN_CFD.length - 1; i++) {
            double[] low = POLLEN_CFD[i];
            double[] high = POLLEN_CFD[i + 1];
            if (spin >= low[0] && spin <= high[0]) {
                double t = (spin - low[0]) / (high[0] - low[0]);
                double cd = low[1] + t * (high[1] - low[1]);
                double cl = low[2] + t * (high[2] - low[2]);
                boolean estimated = (low[0] == 75.0 || high[0] == 75.0);
                boolean unconverged = (low[0] == 150.0 || high[0] == 150.0
                        || low[0] == 600.0 || high[0] == 600.0);
                String note = "Interpolated between " + low[0] + " and " + high[0]
                        + " rad/s. 75 rad/s is estimated; 150 rad/s key is assumed; "
                        + "150 and 600 rad/s runs have convergence concerns.";
                if (ballType == BallType.Big) note += " Nectar is using provisional Pollen coefficients.";
                return new AeroResult(cd, cl, low[0], high[0], false,
                        estimated, unconverged, note);
            }
        }

        return aeroAtPoint(0, spin, true, ballType); // Defensive fallback.
    }

    private static AeroResult aeroAtPoint(int index, double requestedSpin,
                                          boolean clamped, BallType ballType) {
        double[] point = POLLEN_CFD[index];
        boolean estimated = point[0] == 75.0;
        boolean unconverged = point[0] == 150.0 || point[0] == 600.0;
        String note;
        if (point[0] == 0.0) note = "0 rad/s: final-20-row mean";
        else if (point[0] == 75.0) note = "75 rad/s: estimated midpoint; CFD file has no Cd/Cl columns";
        else if (point[0] == 150.0) note = "150 rad/s: assumed from ambiguous filename; run still drifts";
        else if (point[0] == 270.0) note = "270 rad/s: final-20-row mean";
        else if (point[0] == 420.0) note = "420 rad/s: final-20-row mean";
        else if (point[0] == 600.0) note = "600 rad/s: Cd still drifts in final rows";
        else note = "1500 rad/s: final-20-row mean";
        if (clamped) note += " Requested spin is outside CFD table; coefficient was clamped.";
        if (ballType == BallType.Big) note += " Nectar is using provisional Pollen coefficients.";
        return new AeroResult(point[1], point[2], point[0], point[0],
                clamped, estimated, unconverged, note);
    }

    private static double expm1MinusXOverX2(double x) {
        if (Math.abs(x) < 1.0e-4) {
            return 1.0 + x / 3.0 + x * x / 12.0 + x * x * x / 60.0;
        }
        return 2.0 * (Math.exp(x) - 1.0 - x) / (x * x);
    }

    private static double asinh(double x) {
        return Math.log(x + Math.sqrt(x * x + 1.0));
    }

    private static double F(double p) {
        return p * Math.sqrt(1.0 + p * p) + asinh(p);
    }

    private static double H(double p) {
        return p * (2.0 * p * p + 5.0) * Math.sqrt(1.0 + p * p) / 8.0
                + 3.0 / 8.0 * asinh(p);
    }
    // FTC SubsystemBase implementation and existing motor control interfaces.
    @Override
    public void init(RobotHardwareGroup robotHardware) {
        if (robotHardware == null || robotHardware.flywheel == null) {
            panelsTelemetry.addData("[Flywheel] INIT ERROR", "Flywheel motor is missing");
            return;
        }

        flywheel = robotHardware.flywheel;
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        applyPIDFCoefficients(true);
        flywheel.setVelocity(0.0);

        panelsTelemetry.addData("[Flywheel] Init", "Ready - motor in RUN_USING_ENCODER");
        panelsTelemetry.addData("[Flywheel] Encoder ticks/rev", Globals.GOBILDA_5203_6000RPM);
        panelsTelemetry.addData("[Flywheel] Existing P gain", Globals.flywheelKP);
        panelsTelemetry.addData("[Flywheel] Existing F gain", Globals.flywheelKF);
    }

    @Override
    public void update() {
        if (flywheel == null) return;

        if (useDynamicSpeed) {
            updateDynamicRPS();
        }

        applyPIDFCoefficients(false);

        boolean targetAllowed = canSpinFlywheel
                && (!useDynamicSpeed || hasValidDynamicTarget);
        double commandedRPS = targetAllowed
                ? clamp(desiredFlywheelRPS, minimumFlywheelRPM / 60.0,
                maximumFlywheelRPM / 60.0)
                : 0.0;
        double motorTicksPerSecond = commandedRPS
                * MOTOR_REVOLUTIONS_PER_FLYWHEEL_REV
                * Globals.GOBILDA_5203_6000RPM;
        flywheel.setVelocity(motorTicksPerSecond);

        double measuredRPS = getFlywheelRPS();
        double measuredRPM = measuredRPS * 60.0;
        double targetRPM = commandedRPS * 60.0;
        double errorRPM = targetRPM - measuredRPM;
        double errorRPS = errorRPM / 60.0;

        panelsGraph.addData("Flywheel Target RPM", targetRPM);
        panelsGraph.addData("Flywheel Actual RPM", measuredRPM);
        panelsGraph.addData("Flywheel Error RPM", errorRPM);
        panelsGraph.addData("Flywheel Target RPS", commandedRPS);
        panelsGraph.addData("Flywheel Actual RPS", measuredRPS);

        panelsTelemetry.addData("[Flywheel] Enabled", canSpinFlywheel);
        panelsTelemetry.addData("[Flywheel] Dynamic mode", useDynamicSpeed);
        panelsTelemetry.addData("[Flywheel] Dynamic target valid", hasValidDynamicTarget);
        panelsTelemetry.addData("[Flywheel] Commanded target RPM", targetRPM);
        panelsTelemetry.addData("[Flywheel] Measured RPM", measuredRPM);
        panelsTelemetry.addData("[Flywheel] RPM error (target - actual)", errorRPM);
        panelsTelemetry.addData("[Flywheel] Error RPS", errorRPS);
        panelsTelemetry.addData("[Flywheel] Motor power", flywheel.getPower());
        panelsTelemetry.addData("[Flywheel] Motor target ticks/s", motorTicksPerSecond);
        panelsTelemetry.addData("[Flywheel] Motor/flywheel ratio", MOTOR_REVOLUTIONS_PER_FLYWHEEL_REV);
        panelsTelemetry.addData("[Flywheel] PIDF P", Globals.flywheelKP);
        panelsTelemetry.addData("[Flywheel] PIDF I", SHOOTER_KI);
        panelsTelemetry.addData("[Flywheel] PIDF D", SHOOTER_KD);
        panelsTelemetry.addData("[Flywheel] PIDF F", Globals.flywheelKF);
        panelsTelemetry.addData("[Flywheel] Ready for feeder", isReadyForFeed());
    }

    private void applyPIDFCoefficients(boolean force) {
        if (flywheel == null) return;
        boolean changed = force
                || Double.compare(lastAppliedP, Globals.flywheelKP) != 0
                || Double.compare(lastAppliedI, SHOOTER_KI) != 0
                || Double.compare(lastAppliedD, SHOOTER_KD) != 0
                || Double.compare(lastAppliedF, Globals.flywheelKF) != 0;
        if (!changed) return;

        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(Globals.flywheelKP, SHOOTER_KI,
                        SHOOTER_KD, Globals.flywheelKF));
        lastAppliedP = Globals.flywheelKP;
        lastAppliedI = SHOOTER_KI;
        lastAppliedD = SHOOTER_KD;
        lastAppliedF = Globals.flywheelKF;
    }

    public static void setDesiredFlywheelRPS(double rps) {
        if (finite(rps)) {
            desiredFlywheelRPS = clamp(rps, minimumFlywheelRPM / 60.0,
                    maximumFlywheelRPM / 60.0);
        }
    }

    public double getFlywheelRPS() {
        if (flywheel == null || !(Globals.GOBILDA_5203_6000RPM > 0.0)
                || !positive(MOTOR_REVOLUTIONS_PER_FLYWHEEL_REV)) return 0.0;
        return flywheel.getVelocity()
                / (Globals.GOBILDA_5203_6000RPM * MOTOR_REVOLUTIONS_PER_FLYWHEEL_REV);
    }

    public double getFlywheelRPM() {
        return getFlywheelRPS() * 60.0;
    }

    public double getFlywheelErrorRPS() {
        return desiredFlywheelRPS - getFlywheelRPS();
    }

    public double getFlywheelErrorRPM() {
        return getFlywheelErrorRPS() * 60.0;
    }

    public boolean isReadyForFeed() {
        if (flywheel == null || !canSpinFlywheel) return false;
        if (useDynamicSpeed && !hasValidDynamicTarget) return false;
        return Math.abs(getFlywheelErrorRPS()) <= Globals.FLYWHEEL_ERROR_TOL;
    }

    public void runFlywheel() {
        canSpinFlywheel = true;
    }

    public void stopFlywheel() {
        canSpinFlywheel = false;
    }
    private static final class AeroResult {
        final double cd;
        final double cl;
        final double lowerSpinRadPerSec;
        final double upperSpinRadPerSec;
        final boolean tableClamped;
        final boolean estimatedPointUsed;
        final boolean unconvergedPointUsed;
        final String note;

        AeroResult(double cd, double cl, double low, double high,
                   boolean clamped, boolean estimated, boolean unconverged,
                   String note) {
            this.cd = cd;
            this.cl = cl;
            this.lowerSpinRadPerSec = low;
            this.upperSpinRadPerSec = high;
            this.tableClamped = clamped;
            this.estimatedPointUsed = estimated;
            this.unconvergedPointUsed = unconverged;
            this.note = note;
        }
    }

    private static final class ShotSolution {
        boolean valid;
        boolean coefficientsConverged;
        String error = "Unknown solver error";
        double slantRangeM = Double.NaN;
        double horizontalDistanceM = Double.NaN;
        double targetHeightM = Double.NaN;
        double launchAngleRad = Double.NaN;
        double exitSpeedMps = Double.NaN;
        double ballSpinRadPerSec = Double.NaN;
        double flywheelRadPerSec = Double.NaN;
        double flywheelRPM = Double.NaN;
        double cd = Double.NaN;
        double cl = Double.NaN;
        double lowerSpinRadPerSec = Double.NaN;
        double upperSpinRadPerSec = Double.NaN;
        int iterations;
        boolean estimatedPointUsed;
        boolean unconvergedPointUsed;
        boolean tableClamped;
        boolean nectarFallbackUsed;
        String coefficientNote = "No coefficient lookup";
    }

    private static boolean positive(double value) {
        return finite(value) && value > 0.0;
    }

    private static boolean finite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    private static double clamp(double value, double min, double max) {
        if (max < min) return min;
        return Math.max(min, Math.min(max, value));
    }
}
