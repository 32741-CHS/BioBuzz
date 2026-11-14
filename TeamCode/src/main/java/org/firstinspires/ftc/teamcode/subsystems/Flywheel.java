package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.Globals.panelsGraph;
import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.configs.Globals;
import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.BallType;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;


@Configurable
public class Flywheel implements SubsystemBase {

    // Existing public controls: retained so the rest of TeamCode still works.

    private static final double[][] POLLEN_CFD = new double[][]{
            // Spin rad/s, Cd, Cl
            {0.0, 0.586302, 0.101643},
            {75.0, 0.498444, 0.384987},
            {150.0, 0.410585, 0.668330},
            {270.0, 0.982075, 0.869447},
            {420.0, 1.103894, 1.072725},
            {600.0, 0.429050, 1.537901},
            {1500.0, 1.413068, 2.239814}
    };
    private static final double INCH_TO_METRE = 0.0254;
    private static final double TWO_PI = 2.0 * Math.PI;
    public static double desiredFlywheelRPS = 13.0;
    public static double maximumFlywheelRPM = 6000.0;

    // Shooter geometry and model constants.
    public static double minimumFlywheelRPM = 0.0;
    public static boolean useDynamicSpeed = true;
    public static boolean canSpinFlywheel = false;
    public static double CAMERA_HEIGHT_M = 0.37;
    public static double CAMERA_AHEAD_OF_SHOOTER_M = 0.06;
    public static double TARGET_POINT_ABOVE_TAG_CENTRE_M = 0.15;
    public static double POLLEN_RADIUS_M = 0.03556;
    public static double NECTAR_RADIUS_M = 0.04572;
    public static double BALL_SPIN_RATIO = 1.0;
    public static double BALL_INERTIA_BETA = 2.0 / 3.0;
    public static double ENERGY_TRANSFER_EFFICIENCY = 0.70;
    public static double MOTOR_REVOLUTIONS_PER_FLYWHEEL_REV = 1.0;
    /**
     * PLACEHOLDER: replace with flywheel + reflected motor inertia from CAD,
     * referred to the flywheel shaft, in kg*m^2.
     */
    public static double FLYWHEEL_INERTIA_KG_M2 = 0.00025;

    // Reuse the existing Globals P and F values.
    // I and D are adjustable here.
    public static int MAX_COEFFICIENT_ITERATIONS = 20;
    public static double COEFFICIENT_SPEED_TOLERANCE_MPS = 0.001;

    // Pollen CFD data.
    // Spin is in rad/s, NOT RPM.
    public static double TARGET_STALE_SECONDS = 0.35;

    // Nectar uses the Pollen curve provisionally.
    // Nectar's larger radius and its own mass are still used.
    public static double SHOOTER_KI = 0.0;
    public static double SHOOTER_KD = 0.0;
    private final Camera camera;
    private final Drivetrain drivetrain;
    // Existing hardware dependencies.
    private DcMotorEx flywheel;
    private WebcamName insideCam;

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

    // Constructor
    public Flywheel(Drivetrain dt, Camera cam) {
        this.drivetrain = dt;
        this.camera = cam;
    }

    // AprilTag input.

    private static double requiredExitSpeed(
            double d,
            double dh,
            double theta,
            double massKg,
            double radiusM,
            double cd,
            double cl
    ) {

        final double D =
                d * Math.tan(theta) - dh;

        if (!(D > 1.0e-6)) {
            return Double.NaN;
        }

        final double area =
                Math.PI * radiusM * radiusM;

        final double kD =
                Globals.AirPressure * cd * area
                        / (2.0 * massKg);

        final double kL =
                Globals.AirPressure * cl * area
                        / (2.0 * massKg);

        final double p0 =
                Math.tan(theta);

        final double pd =
                2.0 * dh / d - p0;

        final double a =
                2.0 * D / (d * d);

        if (!finite(a) || Math.abs(a) < 1.0e-10) {
            return Double.NaN;
        }

        final double pathLengthTerm =
                (F(p0) - F(pd)) / (2.0 * a);

        if (!(pathLengthTerm > 0.0)
                || !finite(pathLengthTerm)) {

            return Double.NaN;
        }

        final double xi =
                2.0 * kD * pathLengthTerm;

        if (!finite(xi) || xi > 50.0) {
            return Double.NaN;
        }

        final double dragAmplification =
                expm1MinusXOverX2(xi);

        if (!(dragAmplification > 0.0)
                || !finite(dragAmplification)) {

            return Double.NaN;
        }

        final double integralI =
                (
                        Math.pow(1.0 + p0 * p0, 2.5)
                                - Math.pow(1.0 + pd * pd, 2.5)
                ) / 5.0
                        - pd * (H(p0) - H(pd));

        final double magnusRise =
                kL / (a * a)
                        * (
                        5.0 / 3.0 * integralI
                                - Math.pow(
                                1.0 + p0 * p0,
                                1.5
                        ) * (p0 - pd) * (p0 - pd) / 3.0
                );

        final double denominator =
                2.0 * (D + magnusRise);

        if (!(denominator > 1.0e-9)
                || !finite(denominator)) {

            return Double.NaN;
        }

        final double cosTheta =
                Math.cos(theta);

        final double speedSquared =
                Globals.Gravity * d * d
                        * dragAmplification
                        / denominator
                        / (cosTheta * cosTheta);

        if (!(speedSquared > 0.0)
                || !finite(speedSquared)) {

            return Double.NaN;
        }

        return Math.sqrt(speedSquared);
    }

    private static AeroResult getCoefficients(
            BallType ballType,
            double spinRadPerSec
    ) {

        final double spin =
                Math.abs(spinRadPerSec);

        final double firstSpin =
                POLLEN_CFD[0][0];

        final double lastSpin =
                POLLEN_CFD[POLLEN_CFD.length - 1][0];

        if (!finite(spin)) {

            return new AeroResult(
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    false,
                    false,
                    false,
                    "Spin value is not finite"
            );
        }

        for (int i = 0; i < POLLEN_CFD.length; i++) {

            if (Math.abs(spin - POLLEN_CFD[i][0]) < 1.0e-9) {

                return aeroAtPoint(
                        i,
                        spin,
                        false,
                        ballType
                );
            }
        }

        if (spin <= firstSpin) {

            return aeroAtPoint(
                    0,
                    spin,
                    spin < firstSpin,
                    ballType
            );
        }

        if (spin >= lastSpin) {

            return aeroAtPoint(
                    POLLEN_CFD.length - 1,
                    spin,
                    spin > lastSpin,
                    ballType
            );
        }

        for (int i = 0; i < POLLEN_CFD.length - 1; i++) {

            double[] low = POLLEN_CFD[i];
            double[] high = POLLEN_CFD[i + 1];

            if (spin >= low[0] && spin <= high[0]) {

                double t =
                        (spin - low[0])
                                / (high[0] - low[0]);

                double cd =
                        low[1] + t * (high[1] - low[1]);

                double cl =
                        low[2] + t * (high[2] - low[2]);

                boolean estimated =
                        (low[0] == 75.0 || high[0] == 75.0);

                boolean unconverged =
                        (low[0] == 150.0 || high[0] == 150.0
                                || low[0] == 600.0 || high[0] == 600.0);

                String note =
                        "Interpolated between "
                                + low[0] + " and " + high[0]
                                + " rad/s. 75 rad/s is estimated; "
                                + "150 rad/s key is assumed; "
                                + "150 and 600 rad/s runs have convergence concerns.";

                if (ballType == BallType.Big) {

                    note +=
                            " Nectar is using provisional Pollen coefficients.";
                }

                return new AeroResult(
                        cd,
                        cl,
                        low[0],
                        high[0],
                        false,
                        estimated,
                        unconverged,
                        note
                );
            }
        }

        return aeroAtPoint(
                0,
                spin,
                true,
                ballType
        );
    }

    private static AeroResult aeroAtPoint(
            int index,
            double requestedSpin,
            boolean clamped,
            BallType ballType
    ) {

        double[] point = POLLEN_CFD[index];

        boolean estimated = point[0] == 75.0;

        boolean unconverged =
                point[0] == 150.0 || point[0] == 600.0;

        String note;

        if (point[0] == 0.0) {

            note = "0 rad/s: final-20-row mean";

        } else if (point[0] == 75.0) {

            note = "75 rad/s: estimated midpoint; CFD file has no Cd/Cl columns";

        } else if (point[0] == 150.0) {

            note = "150 rad/s: assumed from ambiguous filename; run still drifts";

        } else if (point[0] == 270.0) {

            note = "270 rad/s: final-20-row mean";

        } else if (point[0] == 420.0) {

            note = "420 rad/s: final-20-row mean";

        } else if (point[0] == 600.0) {

            note = "600 rad/s: Cd still drifts in final rows";

        } else {

            note = "1500 rad/s: final-20-row mean";
        }

        if (clamped) {

            note +=
                    " Requested spin is outside CFD table; coefficient was clamped.";
        }

        if (ballType == BallType.Big) {

            note +=
                    " Nectar is using provisional Pollen coefficients.";
        }

        return new AeroResult(
                point[1],
                point[2],
                point[0],
                point[0],
                clamped,
                estimated,
                unconverged,
                note
        );
    }

    // Dynamic RPM calculation.

    private static double expm1MinusXOverX2(double x) {

        if (Math.abs(x) < 1.0e-4) {

            return 1.0
                    + x / 3.0
                    + x * x / 12.0
                    + x * x * x / 60.0;
        }

        return 2.0 * (Math.exp(x) - 1.0 - x)
                / (x * x);
    }

    // Physics solver.

    private static double asinh(double x) {

        return Math.log(
                x + Math.sqrt(x * x + 1.0)
        );
    }

    // Closed-form trajectory equation.

    private static double F(double p) {

        return p * Math.sqrt(1.0 + p * p)
                + asinh(p);
    }

    // Aerodynamic coefficient interpolation.

    private static double H(double p) {

        return p * (2.0 * p * p + 5.0)
                * Math.sqrt(1.0 + p * p) / 8.0
                + 3.0 / 8.0 * asinh(p);
    }

    public static void setDesiredFlywheelRPS(double rps) {

        if (finite(rps)) {

            desiredFlywheelRPS =
                    clamp(
                            rps,
                            minimumFlywheelRPM / 60.0,
                            maximumFlywheelRPM / 60.0
                    );
        }
    }

    // Mathematical helper functions.

    private static boolean finite(double value) {

        return !Double.isNaN(value)
                && !Double.isInfinite(value);
    }

    private static double clamp(
            double value,
            double min,
            double max
    ) {

        if (max < min) {
            return min;
        }

        return Math.max(
                min,
                Math.min(max, value)
        );
    }

    /**
     * Backwards-compatible method used by the existing GoalTracker.
     * <p>
     * distance = FTC ftcPose.range in inches (horizontal range).
     * zRadians = elevation in radians.
     */
    public void changeGoal(double distance, double zRadians) {
        changeGoal(distance, 0.0, zRadians);
    }

    /**
     * Bearing-aware version.
     * <p>
     * distance = FTC horizontal range in inches.
     * bearing = horizontal bearing in radians.
     * elevation = vertical elevation in radians.
     */
    public void changeGoal(
            double distance,
            double bearingRad,
            double elevationRad
    ) {
        this.distanceInput = distance;
        this.bearingRadians = bearingRad;
        this.elevationRadians = elevationRad;

        this.inputIsDirectSlantRange = false;
        this.lastGoalUpdateNanos = System.nanoTime();
    }

    // FTC subsystem implementation.

    /**
     * Direct slant-range API.
     * <p>
     * directSlantRangeM = direct camera-to-tag distance in metres.
     */
    public void changeGoalFromSlantRange(
            double directSlantRangeM,
            double bearingRad,
            double elevationRad
    ) {
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
                || (System.nanoTime() - lastGoalUpdateNanos) * 1.0e-9
                > TARGET_STALE_SECONDS) {

            hasValidDynamicTarget = false;

            panelsTelemetry.addData(
                    "[Flywheel] Dynamic target",
                    "Waiting for a fresh AprilTag"
            );

            return;
        }

        if (!finite(distanceInput)
                || distanceInput <= 0.0
                || !finite(elevationRadians)
                || !finite(bearingRadians)) {

            hasValidDynamicTarget = false;

            panelsTelemetry.addData(
                    "[Flywheel] Solver",
                    "Invalid range/bearing/elevation input"
            );

            return;
        }

        // Preserve the existing colour-camera selection.

        if (camera != null && insideCam != null) {

            BallType detected = camera.getBallType(insideCam);

            if (detected != null && detected != BallType.None) {
                currentBallType = detected;
            }
        }

        lastSolution = solveShot(
                distanceInput,
                bearingRadians,
                elevationRadians,
                inputIsDirectSlantRange,
                currentBallType
        );

        panelsTelemetry.addData(
                "[Flywheel] Ball type",
                currentBallType.name()
        );

        panelsTelemetry.addData(
                "[Flywheel] Range input",
                distanceInput
        );

        panelsTelemetry.addData(
                "[Flywheel] Range input unit",
                inputIsDirectSlantRange
                        ? "metres, direct slant range"
                        : "inches, FTC horizontal range"
        );

        panelsTelemetry.addData(
                "[Flywheel] Bearing deg",
                Math.toDegrees(bearingRadians)
        );

        panelsTelemetry.addData(
                "[Flywheel] Elevation deg",
                Math.toDegrees(elevationRadians)
        );

        if (!lastSolution.valid) {

            hasValidDynamicTarget = false;

            panelsTelemetry.addData(
                    "[Flywheel] Solver status",
                    "INVALID: " + lastSolution.error
            );

            return;
        }

        double rpmBeforeLimit = lastSolution.flywheelRPM;

        double rpmLimited = clamp(
                rpmBeforeLimit,
                minimumFlywheelRPM,
                maximumFlywheelRPM
        );

        desiredFlywheelRPS = rpmLimited / 60.0;

        hasValidDynamicTarget = true;

        // Geometry telemetry.

        panelsTelemetry.addData(
                "[Flywheel] Direct slant range m",
                lastSolution.slantRangeM
        );

        panelsTelemetry.addData(
                "[Flywheel] Horizontal shot distance m",
                lastSolution.horizontalDistanceM
        );

        panelsTelemetry.addData(
                "[Flywheel] Target height above shooter m",
                lastSolution.targetHeightM
        );

        panelsTelemetry.addData(
                "[Flywheel] Launch angle deg",
                Math.toDegrees(lastSolution.launchAngleRad)
        );

        panelsTelemetry.addData(
                "[Flywheel] Required ball exit speed m/s",
                lastSolution.exitSpeedMps
        );

        panelsTelemetry.addData(
                "[Flywheel] Calculated ball spin rad/s",
                lastSolution.ballSpinRadPerSec
        );

        panelsTelemetry.addData(
                "[Flywheel] Required flywheel rad/s",
                lastSolution.flywheelRadPerSec
        );

        panelsTelemetry.addData(
                "[Flywheel] Calculated flywheel RPM",
                rpmBeforeLimit
        );

        panelsTelemetry.addData(
                "[Flywheel] Applied target RPM",
                rpmLimited
        );

        panelsTelemetry.addData(
                "[Flywheel] RPM limit active",
                Math.abs(rpmBeforeLimit - rpmLimited) > 1.0e-6
        );

        // Aerodynamic telemetry.

        panelsTelemetry.addData(
                "[Flywheel] CFD Cd",
                lastSolution.cd
        );

        panelsTelemetry.addData(
                "[Flywheel] CFD Cl",
                lastSolution.cl
        );

        panelsTelemetry.addData(
                "[Flywheel] CFD lower spin rad/s",
                lastSolution.lowerSpinRadPerSec
        );

        panelsTelemetry.addData(
                "[Flywheel] CFD upper spin rad/s",
                lastSolution.upperSpinRadPerSec
        );

        panelsTelemetry.addData(
                "[Flywheel] CFD iterations",
                lastSolution.iterations
        );

        panelsTelemetry.addData(
                "[Flywheel] CFD coefficient convergence",
                lastSolution.coefficientsConverged
        );

        panelsTelemetry.addData(
                "[Flywheel] CFD 75 estimate used",
                lastSolution.estimatedPointUsed
        );

        panelsTelemetry.addData(
                "[Flywheel] CFD unconverged run used",
                lastSolution.unconvergedPointUsed
        );

        panelsTelemetry.addData(
                "[Flywheel] CFD table clamped",
                lastSolution.tableClamped
        );

        panelsTelemetry.addData(
                "[Flywheel] Nectar Pollen fallback",
                lastSolution.nectarFallbackUsed
        );

        panelsTelemetry.addData(
                "[Flywheel] CFD note",
                lastSolution.coefficientNote
        );

        if (Math.abs(rpmBeforeLimit - rpmLimited) > 1.0e-6) {

            panelsTelemetry.addData(
                    "[Flywheel] WARNING",
                    "Calculated RPM exceeded configured motor limit"
            );
        }
    }
    // PIDF configuration.

    private ShotSolution solveShot(
            double inputDistance,
            double bearingRad,
            double elevationRad,
            boolean inputIsSlant,
            BallType ballType
    ) {

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

        // Reconstruct horizontal and vertical distances.

        if (inputIsSlant) {

            slantRangeM = inputDistance;

            horizontalCameraRangeM = slantRangeM
                    / Math.sqrt(
                    1.0 + bearingCos * bearingCos
                            * elevationTan * elevationTan
            );

        } else {

            horizontalCameraRangeM =
                    inputDistance * INCH_TO_METRE;

            double forwardFromCameraM =
                    horizontalCameraRangeM * bearingCos;

            double verticalFromCameraM =
                    forwardFromCameraM * elevationTan;

            slantRangeM = Math.hypot(
                    horizontalCameraRangeM,
                    verticalFromCameraM
            );
        }

        if (!positive(horizontalCameraRangeM)
                || !positive(slantRangeM)) {

            result.error = "Range must be positive";

            return result;
        }

        // Camera-frame target vector.

        final double targetForwardFromCameraM =
                horizontalCameraRangeM * bearingCos;

        final double targetLeftFromCameraM =
                horizontalCameraRangeM * Math.sin(bearingRad);

        final double targetForwardFromShooterM =
                targetForwardFromCameraM
                        + CAMERA_AHEAD_OF_SHOOTER_M;

        final double horizontalDistanceM = Math.hypot(
                targetForwardFromShooterM,
                targetLeftFromCameraM
        );

        // Reconstruct tag-centre height.

        final double cameraToTagVerticalM =
                targetForwardFromCameraM * elevationTan;

        final double targetHeightAboveShooterM =
                cameraToTagVerticalM
                        + CAMERA_HEIGHT_M
                        - Globals.InitialCamHeight
                        + TARGET_POINT_ABOVE_TAG_CENTRE_M;

        // Shooting angle is stored as degrees.

        final double launchAngleRad =
                Math.toRadians(Globals.ShootingAngle);

        if (!positive(horizontalDistanceM)) {

            result.error = "Horizontal shot distance is invalid";

            return result;
        }

        if (!finite(targetHeightAboveShooterM)) {

            result.error = "Target height is invalid";

            return result;
        }

        if (!positive(Globals.AirPressure)
                || !positive(FLYWHEEL_INERTIA_KG_M2)
                || !positive(MOTOR_REVOLUTIONS_PER_FLYWHEEL_REV)
                || !(ENERGY_TRANSFER_EFFICIENCY > 0.0
                && ENERGY_TRANSFER_EFFICIENCY <= 1.0)
                || !positive(BALL_SPIN_RATIO)
                || !positive(BALL_INERTIA_BETA)) {

            result.error =
                    "Check air density, inertia, efficiency, spin ratio and beta settings";

            return result;
        }

        final boolean isPollen = ballType == BallType.Small;

        final double ballMassKg =
                isPollen ? Globals.PollenMass : Globals.NectarMass;

        final double ballRadiusM =
                isPollen ? POLLEN_RADIUS_M : NECTAR_RADIUS_M;

        final double flywheelRadiusM =
                Globals.FlywheelDiameter / 2.0;

        if (!positive(ballMassKg)
                || !positive(ballRadiusM)
                || !positive(flywheelRadiusM)) {

            result.error =
                    "Ball mass/radius or flywheel radius is invalid";

            return result;
        }

        final double d = horizontalDistanceM;
        final double dh = targetHeightAboveShooterM;
        final double theta = launchAngleRad;

        final double D =
                d * Math.tan(theta) - dh;

        if (!(D > 1.0e-6)) {

            result.error =
                    "Target is above the reachable launch line at the configured hood angle";

            return result;
        }

        if (Math.abs(Math.cos(theta)) < 1.0e-6) {

            result.error =
                    "Launch angle is too close to vertical";

            return result;
        }

        double speedGuess =
                Math.sqrt(Globals.Gravity * d * d / (2.0 * D))
                        / Math.cos(theta);

        if (!positive(speedGuess)) {

            result.error =
                    "Could not calculate initial ball exit speed";

            return result;
        }

        double exitSpeed = Double.NaN;
        double spinRadPerSec = Double.NaN;

        AeroResult aero = null;

        boolean converged = false;

        int iterations = 0;

        final int safeIterationLimit =
                Math.max(1, Math.min(64, MAX_COEFFICIENT_ITERATIONS));

        for (int i = 0; i < safeIterationLimit; i++) {

            iterations = i + 1;

            spinRadPerSec =
                    BALL_SPIN_RATIO * speedGuess / ballRadiusM;

            aero = getCoefficients(
                    ballType,
                    spinRadPerSec
            );

            // Original project rotation direction.

            double signedCl =
                    aero.cl * (
                            Globals.RotationalDirection < 0.0
                                    ? 1.0
                                    : -1.0
                    );

            double candidateSpeed = requiredExitSpeed(
                    d,
                    dh,
                    theta,
                    ballMassKg,
                    ballRadiusM,
                    aero.cd,
                    signedCl
            );

            if (!positive(candidateSpeed)) {

                result.error =
                        "Trajectory equation failed; check launch angle, target height and CFD values";

                return result;
            }

            double error =
                    Math.abs(candidateSpeed - speedGuess);

            exitSpeed = candidateSpeed;

            if (error <= COEFFICIENT_SPEED_TOLERANCE_MPS) {

                converged = true;

                speedGuess = candidateSpeed;

                break;
            }

            // Damping prevents excessive oscillation.

            speedGuess =
                    0.5 * speedGuess + 0.5 * candidateSpeed;
        }

        // Re-evaluate at final speed.

        spinRadPerSec =
                BALL_SPIN_RATIO * speedGuess / ballRadiusM;

        aero = getCoefficients(
                ballType,
                spinRadPerSec
        );

        double signedCl =
                aero.cl * (
                        Globals.RotationalDirection < 0.0
                                ? 1.0
                                : -1.0
                );

        exitSpeed = requiredExitSpeed(
                d,
                dh,
                theta,
                ballMassKg,
                ballRadiusM,
                aero.cd,
                signedCl
        );

        if (!positive(exitSpeed)) {

            result.error =
                    "Final trajectory evaluation failed";

            return result;
        }

        spinRadPerSec =
                BALL_SPIN_RATIO * exitSpeed / ballRadiusM;

        converged = converged
                || Math.abs(exitSpeed - speedGuess)
                <= COEFFICIENT_SPEED_TOLERANCE_MPS;

        // Flywheel energy-transfer equation.

        final double S = BALL_SPIN_RATIO;

        final double wheelRadPerSec =
                exitSpeed * Math.sqrt(
                        Math.pow(
                                (1.0 + S) / flywheelRadiusM,
                                2.0
                        )
                                + ballMassKg
                                * (1.0 + BALL_INERTIA_BETA * S * S)
                                / (
                                ENERGY_TRANSFER_EFFICIENCY
                                        * FLYWHEEL_INERTIA_KG_M2
                        )
                );

        final double wheelRpm =
                wheelRadPerSec * 60.0 / TWO_PI;

        if (!positive(wheelRadPerSec)
                || !positive(wheelRpm)) {

            result.error =
                    "Flywheel RPM equation returned an invalid value";

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

        result.lowerSpinRadPerSec =
                aero.lowerSpinRadPerSec;

        result.upperSpinRadPerSec =
                aero.upperSpinRadPerSec;

        result.iterations = iterations;

        result.coefficientsConverged = converged;

        result.estimatedPointUsed =
                aero.estimatedPointUsed;

        result.unconvergedPointUsed =
                aero.unconvergedPointUsed;

        result.tableClamped = aero.tableClamped;

        result.nectarFallbackUsed = !isPollen;

        result.coefficientNote = aero.note;

        return result;
    }

    // Public methods retained for the rest of RAMO.

    @Override
    public void init(RobotHardwareGroup robotHardware) {

        if (robotHardware == null
                || robotHardware.flywheel == null) {

            panelsTelemetry.addData(
                    "[Flywheel] INIT ERROR",
                    "Flywheel motor is missing"
            );

            return;
        }

        flywheel = robotHardware.flywheel;

        flywheel.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        flywheel.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        flywheel.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        insideCam = robotHardware.insideCam;

        applyPIDFCoefficients(true);

        flywheel.setVelocity(0.0);

        panelsTelemetry.addData(
                "[Flywheel] Init",
                "Ready - motor in RUN_USING_ENCODER"
        );

        panelsTelemetry.addData(
                "[Flywheel] Encoder ticks/rev",
                Globals.GOBILDA_5203_6000RPM
        );

        panelsTelemetry.addData(
                "[Flywheel] Existing P gain",
                Globals.flywheelKP
        );

        panelsTelemetry.addData(
                "[Flywheel] Existing F gain",
                Globals.flywheelKF
        );
    }

    @Override
    public void update() {

        if (flywheel == null) {
            return;
        }

        if (useDynamicSpeed) {

            updateDynamicRPS();
        }

        applyPIDFCoefficients(false);

        boolean targetAllowed =
                canSpinFlywheel
                        && (!useDynamicSpeed || hasValidDynamicTarget);

        double commandedRPS =
                targetAllowed
                        ? clamp(
                        desiredFlywheelRPS,
                        minimumFlywheelRPM / 60.0,
                        maximumFlywheelRPM / 60.0
                )
                        : 0.0;

        double motorTicksPerSecond =
                commandedRPS
                        * MOTOR_REVOLUTIONS_PER_FLYWHEEL_REV
                        * Globals.GOBILDA_5203_6000RPM;

        flywheel.setVelocity(motorTicksPerSecond);

        double measuredRPS = getFlywheelRPS();

        double measuredRPM = measuredRPS * 60.0;

        double targetRPM = commandedRPS * 60.0;

        double errorRPM = targetRPM - measuredRPM;

        double errorRPS = errorRPM / 60.0;

        // Graph telemetry.

        panelsGraph.addData(
                "Flywheel Target RPM",
                targetRPM
        );

        panelsGraph.addData(
                "Flywheel Actual RPM",
                measuredRPM
        );

        panelsGraph.addData(
                "Flywheel Error RPM",
                errorRPM
        );

        panelsGraph.addData(
                "Flywheel Target RPS",
                commandedRPS
        );

        panelsGraph.addData(
                "Flywheel Actual RPS",
                measuredRPS
        );

        // Detailed telemetry.

        panelsTelemetry.addData(
                "[Flywheel] Enabled",
                canSpinFlywheel
        );

        panelsTelemetry.addData(
                "[Flywheel] Dynamic mode",
                useDynamicSpeed
        );

        panelsTelemetry.addData(
                "[Flywheel] Dynamic target valid",
                hasValidDynamicTarget
        );

        panelsTelemetry.addData(
                "[Flywheel] Commanded target RPM",
                targetRPM
        );

        panelsTelemetry.addData(
                "[Flywheel] Measured RPM",
                measuredRPM
        );

        panelsTelemetry.addData(
                "[Flywheel] RPM error (target - actual)",
                errorRPM
        );

        panelsTelemetry.addData(
                "[Flywheel] Error RPS",
                errorRPS
        );

        panelsTelemetry.addData(
                "[Flywheel] Motor power",
                flywheel.getPower()
        );

        panelsTelemetry.addData(
                "[Flywheel] Motor target ticks/s",
                motorTicksPerSecond
        );

        panelsTelemetry.addData(
                "[Flywheel] Motor/flywheel ratio",
                MOTOR_REVOLUTIONS_PER_FLYWHEEL_REV
        );

        panelsTelemetry.addData(
                "[Flywheel] PIDF P",
                Globals.flywheelKP
        );

        panelsTelemetry.addData(
                "[Flywheel] PIDF I",
                SHOOTER_KI
        );

        panelsTelemetry.addData(
                "[Flywheel] PIDF D",
                SHOOTER_KD
        );

        panelsTelemetry.addData(
                "[Flywheel] PIDF F",
                Globals.flywheelKF
        );

        panelsTelemetry.addData(
                "[Flywheel] Ready for feeder",
                isReadyForFeed()
        );
    }

    private void applyPIDFCoefficients(boolean force) {

        if (flywheel == null) {
            return;
        }

        boolean changed =
                force
                        || Double.compare(
                        lastAppliedP,
                        Globals.flywheelKP
                ) != 0
                        || Double.compare(
                        lastAppliedI,
                        SHOOTER_KI
                ) != 0
                        || Double.compare(
                        lastAppliedD,
                        SHOOTER_KD
                ) != 0
                        || Double.compare(
                        lastAppliedF,
                        Globals.flywheelKF
                ) != 0;

        if (!changed) {
            return;
        }

        flywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(
                        Globals.flywheelKP,
                        SHOOTER_KI,
                        SHOOTER_KD,
                        Globals.flywheelKF
                )
        );

        lastAppliedP = Globals.flywheelKP;
        lastAppliedI = SHOOTER_KI;
        lastAppliedD = SHOOTER_KD;
        lastAppliedF = Globals.flywheelKF;
    }

    public double getFlywheelRPM() {

        return getFlywheelRPS() * 60.0;
    }

    public double getFlywheelRPS() {

        if (flywheel == null
                || !(Globals.GOBILDA_5203_6000RPM > 0.0)
                || !positive(MOTOR_REVOLUTIONS_PER_FLYWHEEL_REV)) {

            return 0.0;
        }

        return flywheel.getVelocity()
                / (
                Globals.GOBILDA_5203_6000RPM
                        * MOTOR_REVOLUTIONS_PER_FLYWHEEL_REV
        );
    }

    private static boolean positive(double value) {

        return finite(value) && value > 0.0;
    }

    public double getFlywheelErrorRPM() {

        return getFlywheelErrorRPS() * 60.0;
    }

    public double getFlywheelErrorRPS() {

        return desiredFlywheelRPS - getFlywheelRPS();
    }

    // Internal data classes.

    public boolean isReadyForFeed() {

        if (flywheel == null || !canSpinFlywheel) {
            return false;
        }

        if (useDynamicSpeed && !hasValidDynamicTarget) {
            return false;
        }

        return Math.abs(getFlywheelErrorRPS())
                <= Globals.FLYWHEEL_ERROR_TOL;
    }

    public void runFlywheel() {

        canSpinFlywheel = true;
    }

    // General mathematical utilities.

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

        AeroResult(
                double cd,
                double cl,
                double low,
                double high,
                boolean clamped,
                boolean estimated,
                boolean unconverged,
                String note
        ) {

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
}
