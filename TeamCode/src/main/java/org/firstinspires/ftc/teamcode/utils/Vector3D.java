package org.firstinspires.ftc.teamcode.utils;

/**
 * Vector3D is for 3d vectors, with operations such as - plus, - minus - and normalise. A vector can
 * be created by x y z or a special constructor method - fromMag
 */
public class Vector3D {
    public final double x;
    public final double y;
    public final double z;
    public final double mag;

    /** The horizontal angle in radians (0 is along the positive X-axis) */
    public final double angleRadH;

    /** The vertical angle in radians relative to the XY flat plane */
    public final double angleRadV;

    /**
     * An immutable 3D Vector.
     *
     * @param vX left/right (Positive is Right)
     * @param vY forward/backward (Positive is Forward)
     * @param vZ up/down (Positive is Up)
     */
    public Vector3D(double vX, double vY, double vZ) {
        x = vX;
        y = vY;
        z = vZ;
        mag = Math.sqrt(x * x + y * y + z * z);
        angleRadH = Math.atan2(y, x); // 0 radians points along positive X
        angleRadV = Math.atan2(z, Math.sqrt(x * x + y * y));
    }

    /**
     * @param mag the magnitude
     * @param angleRadH the horizontal angle on the flat plane in radians
     * @param angleRadV the vertical angle of mag to the flat plane in radians
     * @return Vector3D
     */
    public static Vector3D fromMag(double mag, double angleRadH, double angleRadV) {
        double groundComponent = Math.cos(angleRadV) * mag;

        double x = Math.cos(angleRadH) * groundComponent;
        double y = Math.sin(angleRadH) * groundComponent;
        double z = Math.sin(angleRadV) * mag;

        return new Vector3D(x, y, z);
    }

    /**
     * Create a new vector by adding the inputted vector to this vector
     *
     * @param v2 the other vector to add
     * @return Vector3D
     */
    public Vector3D plus(Vector3D v2) {
        return new Vector3D(x + v2.x, y + v2.y, z + v2.z);
    }

    /**
     * Subtract the other vector from this vector
     *
     * @param v2 the other vector
     * @return Vector3D
     */
    public Vector3D minus(Vector3D v2) {
        return new Vector3D(x - v2.x, y - v2.y, z - v2.z);
    }

    /**
     * Return a new vector based on this vector where the vector components (x, y and z) are between
     * 0 and 1
     *
     * @return Vector3D
     */
    public Vector3D normalise() {
        double newX = x == 0 || mag == 0 ? 0 : x / mag;
        double newY = y == 0 || mag == 0 ? 0 : y / mag;
        double newZ = z == 0 || mag == 0 ? 0 : z / mag;
        return new Vector3D(newX, newY, newZ);
    }
}
