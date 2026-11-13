package org.firstinspires.ftc.teamcode.opmodes;

import android.health.connect.datatypes.PowerRecord;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "PID Genetic Tuner", group = "Tests")
public class PIDGeneticTunerOpMode extends LinearOpMode {

    // Change these names to match your Robot Configuration.
    private DcMotorEx Flywheel;

    // PID values currently being tested.
    private double kP = 0.02,  kI = 0.0, kD = 0.0;

    private double force;

    // Target distance for the PID test.
    private static final double TARGET_RPMS = 24.0;

    // Change this to match your wheel diameter, gear ratio, and encoder.
    private static final double TICKS_PER_INCH = 45.0;

    // Safety limits.
    private static final double MAX_TEST_TIME_SECONDS = 5.0;
    private static final double MAX_MOTOR_POWER = 0.6;

    // PID test settings.
    private static final double LOOP_PERIOD_SECONDS = 0.02;
    private static final double SETTLE_ERROR_PERCENT = 0.05;

    // Genetic algorithm settings.
    private static final int POPULATION_SIZE = 10;
    private static final int GENERATIONS = 5;
    private static final double MUTATION_RATE = 0.20;
    private static final double CROSSOVER_RATE = 0.80;

    private static final double[] LOWER_BOUNDS = {
            0.005,  // kP
            0.000,  // kI
            0.000   // kD
    };

    private static final double[] UPPER_BOUNDS = {
            0.080,  // kP
            0.002,  // kI
            0.050   // kD
    };

    private final List<org.firstinspires.ftc.teamcode.opmodes.PIDTestResult> testResults = new ArrayList<>();

    @Override
    public void runOpMode() {
        Flywheel = hardwareMap.get(DcMotorEx.class, RobotHardwareGroup.FLYWHEEL_MOTOR_NAME);

        // Reverse the left-side motors if needed for your drivetrain.
        Flywheel.setDirection(DcMotor.Direction.REVERSE);

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Ready to tune PID");
        telemetry.addData("Target RMPS", TARGET_RPMS  + " RPMS");
        telemetry.addData("Press Start", "Run one PID test");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            org.firstinspires.ftc.teamcode.opmodes.PIDTestResult result = runPIDTest(kP, kI, kD);

            testResults.add(result);

            telemetry.clearAll();
            telemetry.addData("Test complete", true);
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("Final error", result.finalError);
            telemetry.addData("Overshoot", result.overshoot);
            telemetry.addData("Settling time", result.settlingTime);
            telemetry.addData("Fitness", result.fitness);
            telemetry.addData("Press Start", "Run another test");
            telemetry.update();

            // Wait until the user starts another test.
            while (opModeIsActive() && !isStarted()) {
                sleep(50);
            }
        }
    }

    private PIDTestResult runPIDTest(double kP, double kI, double kD) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double targetTicks = TARGET_RPMS  * TICKS_PER_INCH;

        double integral = 0;
        double previousError = 0;
        boolean hasPreviousError = false;

        double startTime = getRuntime();
        double settlingTime = MAX_TEST_TIME_SECONDS;
        double maxOvershoot = 0;

        boolean settled = false;

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double elapsedTime = currentTime - startTime;

            double currentPosition = getAverageEncoderPosition();
            double error = targetTicks - currentPosition;
            double absoluteError = Math.abs(error);

            if (!settled && absoluteError <= targetTicks * SETTLE_ERROR_PERCENT) {
                settled = true;
                settlingTime = elapsedTime;
            }

            if (currentPosition > targetTicks) {
                double overshoot = currentPosition - targetTicks;
                maxOvershoot = Math.max(maxOvershoot, overshoot);
            }

            if (elapsedTime >= MAX_TEST_TIME_SECONDS) {
                break;
            }

            integral += error * LOOP_PERIOD_SECONDS;

            // Prevent integral windup.
            integral = clamp(integral, -100, 100);

            double derivative = 0;

            if (hasPreviousError) {
                derivative = (error - previousError) / LOOP_PERIOD_SECONDS;
            }

            double output =
                    kP * error +
                            kI * integral +
                            kD * derivative;

            output = clamp(output, -MAX_MOTOR_POWER, MAX_MOTOR_POWER);

            setDrivePower(output);

            previousError = error;
            hasPreviousError = true;

            telemetry.addData("PID test running", true);
            telemetry.addData("Target ticks", targetTicks);
            telemetry.addData("Current ticks", currentPosition);
            telemetry.addData("Error", error);
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("Motor power", output);
            telemetry.update();

            sleep((long) (LOOP_PERIOD_SECONDS * 1000));
        }

        setDrivePower(0);

        double finalPosition = getAverageEncoderPosition();
        double finalError = Math.abs(targetTicks - finalPosition);

        double fitness = calculateFitness(
                finalError,
                maxOvershoot,
                settlingTime,
                settled
        );

        return new PIDTestResult(
                kP,
                kI,
                kD,
                finalError,
                maxOvershoot,
                settlingTime,
                fitness
        );
    }

    private void setDrivePower(double output) {
        force = output;
    }

    private double calculateFitness(
            double finalError,
            double overshoot,
            double settlingTime,
            boolean settled
    ) {
        double fitness = 1000;

        fitness -= finalError * 2;
        fitness -= overshoot * 3;
        fitness -= settlingTime * 50;

        if (!settled) {
            fitness -= 200;
        }

        return fitness;
    }

    private double getAverageEncoderPosition() {
        return (
                Math.abs(Flywheel.getCurrentPosition()) +
                        Math.abs(Flywheel.getCurrentPosition()) +
                        Math.abs(Flywheel.getCurrentPosition()) +
                        Math.abs(Flywheel.getCurrentPosition())
        ) / 4.0;
    }

    private void setDrivePower(
    ) {
        Flywheel.setPower(force);
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        Flywheel.setMode(mode);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

