package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Color;

import org.firstinspires.ftc.teamcode.configs.RobotHardware;
import org.firstinspires.ftc.teamcode.configs.Globals;
import org.firstinspires.ftc.teamcode.utils.Vector2D;
import org.firstinspires.ftc.teamcode.utils.PixelProcessor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.visionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

import java.util.HashMap;
import java.util.Map;

public final class Camera {
	private static final Map<WebcamName, PixelProcessor> Processors = new HashMap();
	private static final Map<WebcamName, VisionPortal> Portals = new HashMap();
	private final WebcamName insideCam = RobotHardware.insideCam;

	public enum BallType {
		Big,
		Small,
		None
	};

	// coord should be from (1, 1) to (0, 0) aka it takes percentages
	public final Color getRGB(WebcamName cam, Vector2D coord) {
		if (cam == null) {
			return Color.valueOf(0, 0, 0);
		}

		// init any new cameras
		// this would be better if it were in an `init()` function that
		// gets all webcames and would do this automatically
		if (!Portals.containsKey(cam)) {
			PixelProcessor newprocessor = new PixelProcessor();
			VisionPortal newportal = new VisionPortal.Builder()
					.setCamera(cam)
					.addProcessor(newprocessor)
					.build();

			Processors.put(cam, newprocessor);
			Portals.put(cam, newportal);
		}

		PixelProcessor proc = Processors.get(cam);
		if (proc != null) {
			proc.setCoordinates(coord.x, coord.y);
			return proc.getLastColor();
		}

		return Color.valueOf(0, 0, 0);
	};

	// this assumes balls are in the middle of the webcam
	public final BallType getBallType() {
		Vector2D coord = new Vector2D(0.5, 0.5);
		Color color = getRGB(insideCam, coord);
		if (color.green() + color.blue() / 2.0f >= Globals.PollenThreshold) {
			return BallType.Small;
		}

		if (Globals.NectarColor == Globals.NectarColorEnum.Red) {
			if (color.red() >= Globals.NectarThreshold)
				return BallType.Big;
		} else if (Globals.NectarColor == Globals.NectarColorEnum.Red) {
			if (color.blue() >= Globals.NectarThreshold)
				return BallType.Big;
		}

		return BallType.None;
	}
}
