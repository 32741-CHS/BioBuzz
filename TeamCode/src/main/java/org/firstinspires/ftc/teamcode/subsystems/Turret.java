package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.PID_and_KalmanFilter;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

@Configurable
public class Turret implements SubsystemBase {
    // Config
    public static double ANGLE_TOLERANCE = Math.toRadians(1.0);
    public static boolean AllowAutoRotate = true;
    private final Drivetrain drivetrain;
    private final PID_and_KalmanFilter pid = new PID_and_KalmanFilter();
    private double CurrentRotation = 0;
    private Vector2D CurrentGoalPosition = null;
    private boolean HasGoal = false;

    public Turret(Drivetrain dt) {
        this.drivetrain = dt;
    }

    public void changeRotation(double rotation) {
        CurrentRotation = rotation;
    }

    /**
     * Calculates absolute field coordinates from camera relative distance and bearing.
     *
     * @param distance       Distance to goal
     * @param bearingRadians Bearing angle from camera in radians
     */
    public void changeGoal(double distance, double bearingRadians) {
        if (drivetrain == null) return;
        HasGoal = true;

        double robotX = drivetrain.getX();
        double robotY = drivetrain.getY();
        double robotHeading = drivetrain.getHeading();

        // FTC bearing: positive = right (clockwise). Field heading increases counter-clockwise.
        double fieldAngleToGoal = robotHeading - bearingRadians;

        double goalX = robotX + distance * Math.cos(fieldAngleToGoal);
        double goalY = robotY + distance * Math.sin(fieldAngleToGoal);

        CurrentGoalPosition = new Vector2D(goalX, goalY);
    }

    public void changeGoal(double distance, double xradians, double zradians) {
        changeGoal(distance, xradians);
    }

    public void changeGoal(Vector2D fieldPosition) {
        if (fieldPosition == null)
            return;
        HasGoal = true;
        CurrentGoalPosition = fieldPosition;
    }

    @Override
    public void init(RobotHardwareGroup robotHardware) {

    }

    @Override
    public void update() {
        if (!AllowAutoRotate) return;
        if (!HasGoal) return;
        if (drivetrain == null || drivetrain.getFollower() == null || CurrentGoalPosition == null) return;

        CurrentRotation = drivetrain.getHeading();
        panelsTelemetry.addData("Turret Rotation", CurrentRotation);

        double dx = CurrentGoalPosition.x - drivetrain.getX();
        double dy = CurrentGoalPosition.y - drivetrain.getY();
        double targetHeading = Math.atan2(dy, dx);

        double turnPower = calcTurnPower(targetHeading);
        drivetrain.turnInput(turnPower);
    }

    private double calcTurnPower(double target) {
        // get anglewrapping to prevent 360 spin when crossing rear boundary
        double angleError = Math.atan2(
                Math.sin(target - CurrentRotation),
                Math.cos(target - CurrentRotation)
        );
        return pid.PIDangle(0, (float) angleError);
    }

}
