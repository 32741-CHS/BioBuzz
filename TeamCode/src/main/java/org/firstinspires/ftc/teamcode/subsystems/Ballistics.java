package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.utils.MathEx;

public class Ballistics {
    private static double[] DISTANCE_LOOKUP = {0.71, 1.86, 3.27};
    private static double[] SPEED_LOOKUP =    {37.0, 52.0, 70.0};

    public static double calculateTurretAngle(double tagBearingDeg, double currentAngleDeg) {
        return currentAngleDeg + tagBearingDeg;
    }

    public static double calculateFlywheelRPS(double distance) {
        return MathEx.interp(distance, DISTANCE_LOOKUP, SPEED_LOOKUP);
    }
}
