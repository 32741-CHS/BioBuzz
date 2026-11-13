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

    private static volatile double CurrentRotation = 0;
    private static volatile Vector2D CurrentGoalPosition = null;
    private static volatile boolean HasGoal = false;
    private final Drivetrain drivetrain;
    private final PID_and_KalmanFilter pid = new PID_and_KalmanFilter();

    public Turret(Drivetrain dt) {
        this.drivetrain = dt;
    }

    public static void changeRotation(double rotation) {
        CurrentRotation = rotation;
    }

    public static void changeGoal(double distance, double xradians, double zradians) {
        HasGoal = true;
        double angle = Math.atan2(zradians, xradians);
        Vector2D vec = new Vector2D(
                distance * Math.cos(angle),
                distance * Math.sin(angle));
        CurrentGoalPosition = vec;
    }

    public static void changeGoal(Vector2D vec) {
        if (vec == null)
            return;
        CurrentGoalPosition = vec;
    }

    @Override
    public void init(RobotHardwareGroup robotHardware) {

    }

    @Override
    public void update() {
        if (!HasGoal) return;
        if (drivetrain.getFollower() == null || CurrentGoalPosition == null) return;

        CurrentRotation = drivetrain.getHeading();
        panelsTelemetry.addData("Turret Rotation", CurrentRotation);

        double dx = CurrentGoalPosition.x - drivetrain.getX();
        double dy = CurrentGoalPosition.y - drivetrain.getY();
        double targetHeading = Math.atan2(dy, dx);

        double turnPower = calcTurnPower(targetHeading);
        drivetrain.turnInput(turnPower);
    }

    private double calcTurnPower(double target) {
        return pid.PIDangle((float) CurrentRotation, (float) target);
    }

}