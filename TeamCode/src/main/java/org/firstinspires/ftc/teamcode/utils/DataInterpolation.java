package org.firstinspires.ftc.teamcode.utils;

import java.util.Arrays;

public class DataInterpolation {
    private final double[] inputValues;
    private final double[] outputValues;

    public DataInterpolation(double[][] dataPoints) {
        int len = dataPoints.length;
        inputValues = new double[len];
        outputValues = new double[len];
        // Cause I don't want to deal with the mess that is type conversions
        for (int i = 0; i < len; i++) {
            inputValues[i] = dataPoints[i][0];
            outputValues[i] = dataPoints[i][1];
        }
    }

    public double interpolate(double x) {
        // Handle out-of-bounds: lower than minimum boundary
        if (x <= inputValues[0]) {
            return outputValues[0];
        }
        // Handle out-of-bounds: higher than maximum boundary
        if (x >= inputValues[inputValues.length - 1]) {
            return outputValues[outputValues.length - 1];
        }

        // Find the index of x in the sorted array inputValues
        int index = Arrays.binarySearch(inputValues, x);

        // If exact match is found, return corresponding outputValues value
        if (index >= 0) {
            return outputValues[index];
        }

        // If not found, binarySearch returns (-(insertion point) - 1)
        int insertionPoint = -index - 1;

        // Perform linear interpolation formula: y = y0 + (x - x0) * (y1 - y0) / (x1 - x0)
        double x0 = inputValues[insertionPoint - 1];
        double x1 = inputValues[insertionPoint];
        double y0 = outputValues[insertionPoint - 1];
        double y1 = outputValues[insertionPoint];

        return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
    }
}
