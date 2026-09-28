package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.utils.DataInterpolation;

@Configurable
public class Ballistics {
    public static DataInterpolation speedToDistanceLookUp =
            new DataInterpolation(
                    new double[][] {
                        {0.71, 37.0},
                        {1.4, 43.5},
                        {1.85, 52.5},
                        {2.85, 64.5},
                        {3.2, 70.1},
                        {3.45, 73.5}
                    });

    public static double calculateTurretAngle(double tagBearingDeg, double currentAngleDeg) {
        return currentAngleDeg + tagBearingDeg;
    }

    public static double calculateFlywheelRPS(double distance) {
        return speedToDistanceLookUp.interpolate(distance);
    }
}
