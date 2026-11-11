package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Color;

import org.firstinspires.ftc.teamcode.configs.Globals;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.HashMap;
import java.util.Map;

public final class Camera {
	private final Map<WebcamName, PixelProcessor> Processors = new HashMap<>();
	private final Map<WebcamName, VisionPortal> Portals = new HashMap<>();

	public enum BallType {
		Big,
		Small,
		None
	};

	// always be sure to call this when possible
	// takes varargs (cam1, cam2, cam3) and not an array
	public final void init(WebcamName... cams) {
		if (cams == null) return;

		for ( WebcamName cam : cams ) {
			if (cam == null || Portals.containsKey(cam)) continue;

			PixelProcessor newprocessor = new PixelProcessor();
			VisionPortal newportal = new VisionPortal.Builder()
					.setCamera(cam)
					.addProcessor(newprocessor)
					.build();

			Processors.put(cam, newprocessor);
			Portals.put(cam, newportal);
		}
	}

	// coord should be from (1, 1) to (0, 0) aka it takes percentages
	public final int getRGB(WebcamName cam, Vector2D coord) {
		if (cam == null || coord == null) {
			return Color.BLACK;
		}

		PixelProcessor proc = Processors.get(cam);
		if (proc == null) return Color.BLACK;

		proc.setCoordinates(coord.x, coord.y);

		if (!proc.hasProcessedFrame()) return Color.BLACK;

		return proc.getLastColor();
	};

	// this assumes balls are in the middle of the webcam
	public final BallType getBallType(WebcamName insideCam) {
		Vector2D coord = new Vector2D(0.5, 0.5);
		int color = getRGB(insideCam, coord);
		if ((Color.green(color) + Color.blue(color)) / 2.0f >= Globals.PollenThreshold) {
			return BallType.Small;
		}

		if (Globals.NectarColor == Globals.NectarColorEnum.Red) {
			if (Color.red(color) >= Globals.NectarThreshold)
				return BallType.Big;
		} else if (Globals.NectarColor == Globals.NectarColorEnum.Blue) {
			if (Color.blue(color) >= Globals.NectarThreshold)
				return BallType.Big;
		}

		return BallType.None;
	}

	public void close() {
		for (VisionPortal portal : Portals.values()) {
			portal.close();
		}

		Portals.clear();
		Processors.clear();
	}
}
