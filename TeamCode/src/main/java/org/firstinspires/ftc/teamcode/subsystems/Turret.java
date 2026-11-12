package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.configs.pedro.Constants;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;
import org.firstinspires.ftc.teamcode.utils.Vector2D;
import org.firstinspires.ftc.teamcode.utils.PID_and_KalmanFilter;

import java.lang.Math;
import java.lang.System;

@Configurable
public class Turret implements SubsystemBase {
	private Drivetrain drivetrain;
	private PID_and_KalmanFilter pid = new PID_and_KalmanFilter();
	private static Follower follower;
	private static volatile double CurrentRotation = 0;
	private static volatile Vector2D CurrentGoalPosition = null;

	// Config
	public static double ANGLE_TOLERANCE = Math.toRadians(1.0);

	public Turret(Drivetrain dt) {
		this.drivetrain = dt;
	}

	public static void changeRotation(double rotation) {
		CurrentRotation = rotation;
	};

	public static void changeGoal(double distance, double xradians, double zradians) {
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

	private double calcTurnPower(double target) {
		return pid.PIDangle(CurrentRotation, target);
	}

	@Override
	public void init(RobotHardwareGroup robotHardware) {

	}

	@Override
	public void update() {
		if (follower == null || CurrentGoalPosition == null) {
			return;
		}

		Pose currpose = follower.getPose();
		CurrentRotation = currpose.getHeading();

		double dx = CurrentGoalPosition.x - currpose.getX();
		double dy = CurrentGoalPosition.y - currpose.getY();
		double targetHeading = Math.atan2(dy, dx);

		double turnPower = calcTurnPower(target);
		drivetrain.turnInput(turnPower);
	}

}
