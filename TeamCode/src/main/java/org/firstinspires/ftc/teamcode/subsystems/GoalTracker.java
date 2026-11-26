package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.Globals.isRedAlliance;
import static org.firstinspires.ftc.teamcode.configs.Globals.simpleTelemetry;

import androidx.annotation.Nullable;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.configs.BotHardware;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

@Configurable
public class GoalTracker implements SubsystemBase {
    // config
    public static long GoalMemoryTimeoutMillisecs = 1500;

    private AprilTagProcessor aprilTag;
    private double distanceToGoal = Double.NaN;
    private double xRadians = Double.NaN;
    private double zRadians = Double.NaN;
    private @Nullable AprilTagDetection currBest = null;
    private long lastSeenGoalTimestampMillisecs = 0;

    @Override
    public void init(BotHardware botHardware) {
        if (botHardware == null || botHardware.vision == null) return;

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(botHardware.vision)
                .addProcessor(aprilTag)
                .enableLiveView(false)
                .build();
        PanelsCameraStream.INSTANCE.startStream(visionPortal, 30);
    }

    @Override
    public void update() {
        simpleTelemetry.addData("GoalTracker AprilTagProcessor", aprilTag != null);
        if (aprilTag == null) return;

        distanceToGoal = Double.NaN;
        xRadians = Double.NaN;
        zRadians = Double.NaN;

        AprilTagDetection best = null;

        simpleTelemetry.addData("GoalTracker Detections", aprilTag.getDetections());

        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection == null || detection.ftcPose == null)
                continue;

            String name = getTagName(detection);

            // Alliance filtering if name metadata exists
            if (name != null) {
                if (isRedAlliance && !name.startsWith("RED"))
                    continue;

                if (!isRedAlliance && !name.startsWith("BLUE"))
                    continue;
            }

            // Reject upside-down tags
            if (Math.abs(detection.ftcPose.roll) >= 90)
                continue;

            // Prioritize multi-tag Cluster detections for higher accuracy, or choose closest
            boolean isCluster = detection instanceof AprilTagClusterDetection;
            boolean bestIsCluster = best instanceof AprilTagClusterDetection;

            if (best == null || (isCluster && !bestIsCluster) || (isCluster == bestIsCluster && detection.ftcPose.range < best.ftcPose.range)) {
                best = detection;
            }
        }

        currBest = best;
        if (best == null) {
            simpleTelemetry.addData("GoalTracker Best", "Not found");
            xRadians = 0;
            return;
        } else {
            simpleTelemetry.addData("GoalTracker Best", "Found");
        }

        simpleTelemetry.addData("GoalTracker Best", best);
        lastSeenGoalTimestampMillisecs = System.currentTimeMillis();

        distanceToGoal = best.ftcPose.range;
        xRadians = Math.toRadians(best.ftcPose.bearing);
        zRadians = Math.toRadians(best.ftcPose.elevation);

        simpleTelemetry.addData("GoalTracker Distance", distanceToGoal);
        simpleTelemetry.addData("GoalTracker xRadians", xRadians);
        simpleTelemetry.addData("GoalTracker zRadians", zRadians);
    }

    private String getTagName(AprilTagDetection detection) {
        if (detection instanceof AprilTagSingleDetection) {
            AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
            return single.metadata != null ? single.metadata.name : null;
        } else if (detection instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            return cluster.metadata != null ? cluster.metadata.name : null;
        }
        return null;
    }

    public void setTeam(boolean isRed) {
        isRedAlliance = isRed;
    }

    public double getDistance() {
        return distanceToGoal;
    }

    public double getXRadians() {
        return xRadians;
    }

    public double getZRadians() {
        return zRadians;
    }

    public AprilTagDetection getBest() {
        return currBest;
    }
}
