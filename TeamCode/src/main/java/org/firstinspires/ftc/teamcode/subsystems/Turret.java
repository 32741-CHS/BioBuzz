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

import java.lang.Math;

@Configurable
public class TemplateSubsystem implements SubsystemBase {
	private static Follower follower;
	private static volatile double CurrentRotation = 0;
	private static volatile Vector2D CurrentGoalPosition = null;
	private DcMotor flDrive, frDrive, blDrive, brDrive, feeder, intake;

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

	@Override
	public void init(RobotHardwareGroup robotHardware) {
		flDrive = robotHardware.flDrive;

	}

	@Override
	public void update() {
	}
}
