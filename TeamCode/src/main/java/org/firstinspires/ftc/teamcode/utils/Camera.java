package org.firstinspires.ftc.teamcode.utils;

import android.graphics.color;

import org.firstinspires.ftc.teamcode.configs.RobotHardware;
import org.firstinspires.ftc.teamcode.configs.Globals;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

import org.firstinspires.ftc.robotcore.external.hardware.camera.webcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.visionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

import java.util.HashMap;
import java.util.Map;

public final class Camera {
	private static final Map<webcamName, PixelProcessor> Processors = new HashMap();
	private static final Map<webcamName, VisionPortal> Portals = new HashMap();

	private final webcamName insideCam = RobotHardware.insideCam;
	private VisionPortal Portal = VisionPortal;

	public final enum BallType {
		Big,
		Small,
		None
	};

	public final Color getRGB(webcamName cam, Vector2D coord) {
		if (cam == null) {
			return Color.valueOf(0, 0, 0);
		}

		// init any new cameras
		if (!Portals.containsKey(cam)) {
			PixelProcessor newprocessor = new PixelProcessor();
			VisionPortal newportal = new VisionPortal.builder()
					.setCamera(cam)
					.addProcessor(newProcessor)
					.build();

			Processors.put(cam, newprocessor);
			Portals.put(cam, newportal);
		}

		PixelProcessor proc = Processors.get(cam);
		if (proc != null) {
			proc.setCoordinates((int) coord.x, (int) coord.y);
			return proc.getLastColor();
		}

		return Color.valueOf(0, 0, 0);
	};

	// this assumes balls are in the middle of the webcam
	public final BallType getBallType() {
		// default is 640, 480 for ftc webcams
		int cam_x = 640;
		int cam_y = 480;
		Vector2D coord = new Vector2D(cam_x / 2, cam_y / 2);

		Color color = getRGB(insideCam, coord);
		if (color.yellow() >= Globals.PollenThreshold) {
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
