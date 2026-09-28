package org.firstinspires.ftc.teamcode.utils;

public class Vector2D {
    public final double x;
    public final double y;
    public final double mag;
    public final double angleRad;

    /**
     * An immutable 2D Vector.
     * @param componentX The adjacent side of triangle
     * @param componentY The opposite side of triangle
     */
    public Vector2D(double componentX, double componentY) {
        x = componentX;
        y = componentY;
        mag = Math.sqrt(x * x + y * y);
        angleRad = Math.atan2(y, x);
    }

    /**
     * @param mag the magnitude
     * @param angleRad between x and y
     * @return Vector2D
     */
    public static Vector2D fromMag(double mag, double angleRad) {
        double x = Math.cos(angleRad) * mag;
        double y = Math.sin(angleRad) * mag;

        return new Vector2D(x, y);
    }

    /**
     * Add this vector to the other vector to give a new vector
     * @param v2 The other vector
     * @return Vector2D
     */
    public Vector2D plus(Vector2D v2) {
        return new Vector2D(x + v2.x, y + v2.y);
    }

    /**
     * Subtract this vector from the other vector (IN THIS ORDER) and return a new vector
     * @param v2 other vector
     * @return Vector2D
     */
    public Vector2D minus(Vector2D v2) {
        return new Vector2D(x - v2.x, y - v2.y);
    }

    public Vector2D normalise() {
        double newX = x == 0 || mag == 0 ? 0 : x / mag;
        double newY = y == 0 || mag == 0 ? 0 : y / mag;
        return new Vector2D(newX, newY);
    }
}
