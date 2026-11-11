package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Canvas;
import android.graphics.Color;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

public class PixelProcessor implements VisionProcessor {
	private volatile double normX = 0.5;
	private volatile double normY = 0.5;
	private volatile Color lastColor = Color.valueOf(0, 0, 0);

	public void setCoordinates(double normX, double normY) {
		this.normX = normX;
		this.normY = normY;
	}

	public Color getLastColor() {
		return lastColor;
	}

	@Override
	public void init(int width, int height, CameraCalibration calibration) {
		// Nothing needed here!
	}

	@Override
	public Object processFrame(Mat frame, long captureTimeNanos) {
		// Convert percentages into exact pixel coordinates for whatever resolution is
		// running
		int x = (int) (normX * frame.cols());
		int y = (int) (normY * frame.rows());

		// Clamp values so they never go out of bounds if normX or normY is 1.0
		x = Math.max(0, Math.min(x, frame.cols() - 1));
		y = Math.max(0, Math.min(y, frame.rows() - 1));

		// Read the RGB values from the OpenCV Mat (FTC VisionPortal uses RGB format)
		double[] rgb = frame.get(y, x);
		if (rgb != null && rgb.length >= 3) {
			// Color.valueOf takes floats between 0.0 and 1.0, while OpenCV gives 0 to 255
			lastColor = Color.valueOf(
					(float) (rgb[0] / 255.0),
					(float) (rgb[1] / 255.0),
					(float) (rgb[2] / 255.0));
		}

		return null;
	}

	@Override
	public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
			float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
	}
}
