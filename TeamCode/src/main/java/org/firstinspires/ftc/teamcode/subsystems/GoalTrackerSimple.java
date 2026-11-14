package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

@Configurable
public class GoalTrackerSimple implements SubsystemBase {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean isRed = true;
    private double distanceToGoal = Double.NaN;
    private double xHeading = Double.NaN;
    private double zHeading = Double.NaN;

    @Override
    public void init(RobotHardwareGroup robotHardware) {
        if (robotHardware == null || robotHardware.vision == null) {
            panelsTelemetry.addData("SIMPLE SHOOTER ERROR NO VISION ", 1);
            return;
        }

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(robotHardware.vision)
                .addProcessor(aprilTag)
                .enableLiveView(false)
                .build();
    }

    @Override
    public void update() {
        if (aprilTag == null) return;

        AprilTagDetection best = null;

        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection == null || detection.ftcPose == null) continue;

            String name = getTagName(detection);

            // Alliance filtering if name metadata exists
            if (name != null) {
                if (isRed && !name.startsWith("RED")) continue;
                if (!isRed && !name.startsWith("BLUE")) continue;
            }

            // Reject upside-down tags
            if (Math.abs(detection.ftcPose.roll) >= 90) continue;

            // Prioritize multi-tag Cluster detections for higher accuracy, or choose closest
            boolean isCluster = detection instanceof AprilTagClusterDetection;
            boolean bestIsCluster = best instanceof AprilTagClusterDetection;

            if (best == null || (isCluster && !bestIsCluster) || (isCluster == bestIsCluster && detection.ftcPose.range < best.ftcPose.range)) {
                best = detection;
            }
        }

        if (best == null) return;

        distanceToGoal = best.ftcPose.range;
        xHeading = Math.toRadians(best.ftcPose.bearing);
        zHeading = Math.toRadians(best.ftcPose.elevation);

        panelsTelemetry.addData("GoalTracker Distance", distanceToGoal);
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

    public void setIsTeamRed(boolean isRed) {
        this.isRed = isRed;
    }

    public double getDistance() {
        return distanceToGoal;
    }

    public double getXHeading() {
        return xHeading;
    }

    public double getZHeading() {
        return zHeading;
    }
}
