package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Canvas;
import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.configs.Globals;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

import java.util.HashMap;
import java.util.Map;

public final class Camera {
    private final Map<WebcamName, PixelProcessor> Processors = new HashMap<>();
    private final Map<WebcamName, VisionPortal> Portals = new HashMap<>();

    // always be sure to call this when possible
    // takes varargs (cam1, cam2, cam3) and not an array
    public void init(WebcamName... cams) {
        if (cams == null) return;

        for (WebcamName cam : cams) {
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

    // this assumes balls are in the middle of the webcam
    public BallType getBallType(WebcamName insideCam) {
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

    // coord should be from (1, 1) to (0, 0) aka it takes percentages
    public int getRGB(WebcamName cam, Vector2D coord) {
        if (cam == null || coord == null) {
            return Color.BLACK;
        }

        PixelProcessor proc = Processors.get(cam);
        if (proc == null) return Color.BLACK;

        proc.setCoordinates(coord.x, coord.y);

        if (!proc.hasProcessedFrame()) return Color.BLACK;

        return proc.getLastColor();
    }

    public void close() {
        for (VisionPortal portal : Portals.values()) {
            portal.close();
        }

        Portals.clear();
        Processors.clear();
    }

    public enum BallType {
        Big,
        Small,
        None
    }

    public static class PixelProcessor implements VisionProcessor {
        private volatile Vector2D Coordinates = new Vector2D(0.5, 0.5);
        private volatile int lastColor = Color.BLACK;
        private volatile boolean processedFrame = false;

        public void setCoordinates(double normX, double normY) {
            normX = Math.max(0.0, Math.min(1.0, normX));
            normY = Math.max(0.0, Math.min(1.0, normY));
            Coordinates = new Vector2D(normX, normY);
        }

        public int getLastColor() {
            return lastColor;
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            // Nothing needed here!
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
                    clampChannel(rgb[0]),
                    clampChannel(rgb[1]),
                    clampChannel(rgb[2])
            );
            processedFrame = true;

            return null;
        }

        private static int clampChannel(double value) {
            return (int) Math.max(0, Math.min(255, Math.round(value)));
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        }

        public boolean hasProcessedFrame() {
            return processedFrame;
        }
    }
}
