package org.firstinspires.ftc.teamcode.utils;

import java.util.Arrays;

public class MathEx {

    /**
     * 1D Linear Interpolation replicating numpy.interp behavior.
     *
     * @param x  The x-coordinate at which to evaluate.
     * @param xp The x-coordinates of the data points (must be increasing).
     * @param fp The y-coordinates of the data points (same length as xp).
     * @return The interpolated value.
     */
    public static double interp(double x, double[] xp, double[] fp) {
        // Handle out-of-bounds: lower than minimum boundary
        if (x <= xp[0]) {
            return fp[0];
        }
        // Handle out-of-bounds: higher than maximum boundary
        if (x >= xp[xp.length - 1]) {
            return fp[fp.length - 1];
        }

        // Find the index of x in the sorted array xp
        int index = Arrays.binarySearch(xp, x);

        // If exact match is found, return corresponding fp value
        if (index >= 0) {
            return fp[index];
        }

        // If not found, binarySearch returns (-(insertion point) - 1)
        int insertionPoint = -index - 1;

        // Boundaries for interpolation
        int i0 = insertionPoint - 1;
        int i1 = insertionPoint;

        // Perform linear interpolation formula: y = y0 + (x - x0) * (y1 - y0) / (x1 - x0)
        double x0 = xp[i0];
        double x1 = xp[i1];
        double y0 = fp[i0];
        double y1 = fp[i1];

        return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
    }
}