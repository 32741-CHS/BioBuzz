package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID_and_KalmanFilter {
//in this code we still need to PID tuning with the constants, we also need to find the measured noise covariance

    //Kalman Filter
    private double Uhatx = 0.00; // real believed x position
    private double Px = 1.00;     // x error covariance
    private double Uhaty = 0.00;  // real believed y position
    private double Py = 1.00;     // Y Error covariance

    // PID_system
    private final ElapsedTime timerX = new ElapsedTime();
    private final ElapsedTime timerY = new ElapsedTime();
    //x
    private double preTimeX = 0;
    private double prevErrorX = 0;
    private double cumErrorX = 0;
    //y
    private double preTimeY = 0;
    private double prevErrorY = 0;
    private double cumErrorY = 0;

    // Tuning stuff
    private final double Kp = 2.0;
    private final double Ki = 0.5;
    private final double Kd = 0.1;

    public double kalmanFilterX(float measurement_x_Odometry, float measurement_x_Camera) {
        double RxO = 0.03;  // Measurement noise Covariance for odometry wheels
        double RxC = 0.03;  // Measurement noise Covariance for camera
        double Qx = 0.001;  // Process noise covariance

        // Prediction
        Px = Px + Qx;

        // Update odometry wheels
        double KL = Px / (Px + RxO);
        Uhatx = Uhatx + KL * (measurement_x_Odometry - Uhatx);
        Px = (1 - KL) * Px;

        // Update Camera
        double KM = Px / (Px + RxC);
        Uhatx = Uhatx + KM * (measurement_x_Camera - Uhatx);
        Px = (1 - KM) * Px; // Added missing covariance update step for camera

        return Uhatx;
    }

    public double kalmanFilterY(float measurement_y_Odometry, float measurement_y_Camera) {
        double RyO = 0.03;
        double RyC = 0.03;
        double Qy = 0.001;

        // Prediction
        Py = Py + Qy;

        // Update odometry wheels
        double KL = Py / (Py + RyO);
        Uhaty = Uhaty + KL * (measurement_y_Odometry - Uhaty);
        Py = (1 - KL) * Py;

        // Update camera
        double KM = Py / (Py + RyC);
        Uhaty = Uhaty + KM * (measurement_y_Camera - Uhaty);
        Py = (1 - KM) * Py;

        return Uhaty;
    }

    public float PIDX(float realVal, float setpoint) {
        double currentTime = timerX.seconds();
        double elapsedTime = currentTime - preTimeX;

        // Run calculation only if enough time has passed to prevent divide by zero or jitter this just makes it smoother
        if (elapsedTime >= 0.01) {
            double error = setpoint - realVal;
            cumErrorX += error * elapsedTime;
            double rateError = (error - prevErrorX) / elapsedTime;

            cumErrorX = Range.clip(cumErrorX, -1000, 1000);
            double output = (Kp * error) + (Ki * cumErrorX) + (Kd * rateError);

            // Clip to motor powers (-1.0 to 1.0) can change the range if you want
            output = Range.clip(output, -1.0, 1.0);

            // Save states for next loop cycle of these values
            prevErrorX = error;
            preTimeX = currentTime;

            return (float) output;
        }

        // Return previous cached calculation if called too quickly to avoid dead zones of no value
        return (float) Range.clip((Kp * (setpoint - realVal)) + (Ki * cumErrorX), -1.0, 1.0);
    }

    public float PIDY(float realVal, float setpoint) {
        double currentTime = timerY.seconds();
        double elapsedTime = currentTime - preTimeY;

        if (elapsedTime >= 0.01) {
            double error = setpoint - realVal;
            cumErrorY += error * elapsedTime;
            double rateError = (error - prevErrorY) / elapsedTime;

            cumErrorY = Range.clip(cumErrorY, -1000, 1000);
            double output = (Kp * error) + (Ki * cumErrorY) + (Kd * rateError);

            output = Range.clip(output, -1.0, 1.0);

            prevErrorY = error;
            preTimeY = currentTime;

            return (float) output;
        }

        return (float) Range.clip((Kp * (setpoint - realVal)) + (Ki * cumErrorY), -1.0, 1.0);
    }
}
