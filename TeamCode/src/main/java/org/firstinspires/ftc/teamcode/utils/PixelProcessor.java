package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Canvas;
import android.graphics.Color;
import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.teamcode.utils.Vector2D;
import org.opencv.core.Mat;

public class PixelProcessor implements VisionProcessor {
	private volatile Vector2D Coordinates = new Vector2D(0, 0);
	private volatile int lastColor = Color.BLACK;
	private volatile boolean processedFrame = false;

	public void setCoordinates(double normX, double normY) {
		Coordinates = new Vector2D(normX, normY);
	}

	public int getLastColor() {
		return lastColor;
	}

	@Override
	public void init(int width, int height, CameraCalibration calibration) {
		// Nothing needed here!
	}

	public boolean hasProcessedFrame() {
		return processedFrame;
	}

    @Override
	public Object processFrame(Mat frame, long captureTimeNanos) {
		if (frame == null || frame.empty()
				|| frame.cols() <= 0
				|| frame.rows() <= 0
				|| frame.channels() < 3) {
			return null;
		}

		Vector2D target = Coordinates;
		// Convert percentages into exact pixel coordinates for whatever resolution is
		// running
		int x = (int) (target.x * frame.cols());
		int y = (int) (target.y * frame.rows());


		// Clamp values so they never go out of bounds if normX or normY is 1.0
		x = Math.max(0, Math.min(x, frame.cols() - 1));
		y = Math.max(0, Math.min(y, frame.rows() - 1));

		// Read the RGB values from the OpenCV Mat (FTC VisionPortal uses RGB format)
		double[] rgb = frame.get(y, x);
		if (rgb == null || rgb.length < 3) return null;

		// returns int as always
		lastColor = Color.rgb(
				(int) Math.round(rgb[0]),
				(int) Math.round(rgb[1]),
				(int) Math.round(rgb[2])
		);
		processedFrame = true;

		return null;
	}

	@Override
	public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
			float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
	}
}
