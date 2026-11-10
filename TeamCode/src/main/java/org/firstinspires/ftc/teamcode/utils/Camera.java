package org.firstinspires.ftc.teamcode.utils;

import android.graphics.color;

import org.firstinspires.ftc.teamcode.configs.RobotHardware;

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

	public final Color getRGB(webcamName cam, float x, float y) {
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
		if (!proc == null) {
			proc.setCoordinates(x, y);
			return proc.getLastColor();
		}

		return Color.valueOf(0, 0, 0);
	};

	public final BallType getBallType() {

	}
}
