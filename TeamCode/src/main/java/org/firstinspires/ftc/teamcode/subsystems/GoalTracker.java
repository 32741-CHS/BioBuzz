package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class GoalTracker implements SubsystemBase {

    private AprilTagProcessor aprilTag;

    private boolean isRed = true;

    private double distanceToGoal = Double.NaN;
    private double xRadians = Double.NaN;
    private double zRadians = Double.NaN;

    @Override
    public void init(RobotHardwareGroup robotHardware) {

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(robotHardware.vision)
                .addProcessor(aprilTag)
                .build();
    }

    @Override
    public void update() {
        distanceToGoal = Double.NaN;
        xRadians = Double.NaN;
        zRadians = Double.NaN;

        AprilTagClusterDetection best = null;

        for (AprilTagDetection detection : aprilTag.getDetections()) {

            if (!(detection instanceof AprilTagClusterDetection))
                continue;

            AprilTagClusterDetection cluster =
                    (AprilTagClusterDetection) detection;

            if (cluster.metadata == null || cluster.ftcPose == null)
                continue;

            // Alliance filtering
            String name = cluster.metadata.name;

            if (isRed && !name.startsWith("RED"))
                continue;

            if (!isRed && !name.startsWith("BLUE"))
                continue;

            // Reject upside-down cells
            if (Math.abs(cluster.ftcPose.roll) >= 90)
                continue;

            // Choose closest valid cluster
            if (best == null ||
                    cluster.ftcPose.range < best.ftcPose.range) {
                best = cluster;
            }
        }

        if (best == null)
            return;

        distanceToGoal = best.ftcPose.range;
        xRadians = Math.toRadians(best.ftcPose.bearing);
        zRadians = Math.toRadians(best.ftcPose.elevation);
    }

    public void setTeam(boolean isRed) {
        this.isRed = isRed;
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
}
