package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.bylazar.camerastream.PanelsCameraStream;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.configs.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


// 20 = blue goal, 24 = red goal
public class Vision {
    private AprilTagProcessor processor;
    private VisionPortal portal;

    public static final int BLUE_GOAL_TAG = 20;
    public static final int RED_GOAL_TAG = 24;

    public static int MAX_FPS = 15;
    public Vision(RobotHardware hw) {
        processor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hw.vision)
                .setCameraResolution(new Size(640, 480))
                .addProcessor(processor)
                .build();

        processor.setDecimation(3);

        while (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            // wait for camera to be ready
        }

        ExposureControl exp = portal.getCameraControl(ExposureControl.class);
        if (exp != null) {
            exp.setMode(ExposureControl.Mode.Manual);
            exp.setExposure(2, TimeUnit.MILLISECONDS);
        }

        GainControl gain = portal.getCameraControl(GainControl.class);
        if (gain != null) gain.setGain(80);

        PanelsCameraStream.INSTANCE.startStream(portal, MAX_FPS);
    }

    public List<AprilTagDetection> getDetections() {
        return processor.getDetections();
    }

    // check if a specific goal tag is visible
    public boolean isTagVisible(int tagId) {
        return getTagById(tagId) != null;
    }

    // returns 0 if tag not visible.
    public double getTagBearing(int tagId) {
        AprilTagDetection tag = getTagById(tagId);
        if (tag == null) return 0;
        return Math.toDegrees(tag.ftcPose.bearing);
    }

    public double getTagRange(int tagId) {
        AprilTagDetection tag = getTagById(tagId);
        if (tag == null) return -1;
        return tag.ftcPose.range;
    }

    public double getTagYaw(int tagId) {
        AprilTagDetection tag = getTagById(tagId);
        if (tag == null) return 0;
        return Math.toDegrees(tag.ftcPose.yaw);
    }

    public AprilTagDetection getTagById(int tagId) {
        List<AprilTagDetection> detections = getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == tagId) {
                return detection;
            }
        }
        return null;
    }

    // get the goal tag ID based on alliance color
    // TODO: we need to have a way to set alliance (manual for now via controller)
    public int getGoalTagId(boolean isRedAlliance) {
        return isRedAlliance ? RED_GOAL_TAG : BLUE_GOAL_TAG;
    }

    public void stop() {
        if (portal != null) {
            portal.close();
        }
    }
}
