package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.utils.MathEx;

@Configurable
public class Ballistics {
    public static double[] DISTANCE_LOOKUP = {0.71, 1.86, 3.2}; // metres
    public static double[] SPEED_LOOKUP =    {37.0, 52.0, 70.1}; // rps

    public static double calculateTurretAngle(double tagBearingDeg, double currentAngleDeg) {
        return currentAngleDeg + tagBearingDeg;
    }

    public static double calculateFlywheelRPS(double distance) {
        return MathEx.interp(distance, DISTANCE_LOOKUP, SPEED_LOOKUP);
    }
}
