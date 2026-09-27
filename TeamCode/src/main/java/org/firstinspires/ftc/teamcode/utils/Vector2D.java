package org.firstinspires.ftc.teamcode.utils;

public class Vector2D {
    public final double x;
    public final double y;
    public final double mag;
    public final double angleRad;
    public final double angleDeg;

    /** An immutable 2D Vector. */
    public Vector2D(double vX, double vY) {
        x = vX;
        y = vY;
        mag = Math.sqrt(x * x + y * y);
        angleRad = Math.atan2(y, x);
        angleDeg = Math.toDegrees(angleRad);
    }

    /**
     * @param mag the magnitude
     * @param deg the horizontal angle on the flat plane
     * @return Vector2D
     */
    public static Vector2D fromMag(double mag, double deg) {
        double rad = Math.toRadians(deg);

        double x = Math.cos(rad) * mag;
        double y = Math.sin(rad) * mag;

        return new Vector2D(x, y);
    }

    public Vector2D plus(Vector2D v2) {
        return new Vector2D(x + v2.x, y + v2.y);
    }

    public Vector2D minus(Vector2D v2) {
        return new Vector2D(x - v2.x, y - v2.y);
    }

    public Vector2D normalise() {
        double newX = x == 0 || mag == 0 ? 0 : x / mag;
        double newY = y == 0 || mag == 0 ? 0 : y / mag;
        return new Vector2D(newX, newY);
    }
}
