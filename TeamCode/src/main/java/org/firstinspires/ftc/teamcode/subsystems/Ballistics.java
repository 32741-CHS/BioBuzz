package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.utils.MathEx;

@Configurable
public class Ballistics {
    public static double[] DISTANCE_LOOKUP = {0.71, 1.4, 1.85, 2.85, 3.2, 3.45}; // metres
    public static double[] SPEED_LOOKUP =    {37.0, 43.5, 52.5, 64.5, 70.1, 73.5}; // rps

    public static double calculateTurretAngle(double tagBearingDeg, double currentAngleDeg) {
        return currentAngleDeg + tagBearingDeg;
    }

    public static double calculateFlywheelRPS(double distance) {
        return MathEx.interp(distance, DISTANCE_LOOKUP, SPEED_LOOKUP);
    }
}
