package org.firstinspires.ftc.teamcode.opmodes;

public class PIDTestResult {

    public final double kP;
    public final double kI;
    public final double kD;

    public final double finalError;
    public final double overshoot;
    public final double settlingTime;
    public final double fitness;

    public PIDTestResult(
            double kP,
            double kI,
            double kD,
            double finalError,
            double overshoot,
            double settlingTime,
            double fitness
    ) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.finalError = finalError;
        this.overshoot = overshoot;
        this.settlingTime = settlingTime;
        this.fitness = fitness;
    }
}

