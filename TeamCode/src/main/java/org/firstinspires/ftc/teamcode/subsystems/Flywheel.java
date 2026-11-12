package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.Globals.AerodynamicDrag;
import static org.firstinspires.ftc.teamcode.configs.Globals.BallGeneralMass;
import static org.firstinspires.ftc.teamcode.configs.Globals.DecaySpin;
import static org.firstinspires.ftc.teamcode.configs.Globals.FLYWHEEL_ERROR_TOL;
import static org.firstinspires.ftc.teamcode.configs.Globals.FlywheelDiameter;
import static org.firstinspires.ftc.teamcode.configs.Globals.GOBILDA_5203_6000RPM;
import static org.firstinspires.ftc.teamcode.configs.Globals.Gravity;
import static org.firstinspires.ftc.teamcode.configs.Globals.InitialCamHeight;
import static org.firstinspires.ftc.teamcode.configs.Globals.RotationalDirection;
import static org.firstinspires.ftc.teamcode.configs.Globals.ShootingAngle;
import static org.firstinspires.ftc.teamcode.configs.Globals.SimulationTimeout;
import static org.firstinspires.ftc.teamcode.configs.Globals.TimeInterval;
import static org.firstinspires.ftc.teamcode.configs.Globals.panelsGraph;
import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.configs.Globals;
import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

@Configurable
public class Flywheel implements SubsystemBase {
	public static double desiredFlywheelRPS = 13;
	public static double maximumFlywheelRPM = 6000;
	public static double minimumFlywheelRPM = 0;
	public static boolean useDynamicSpeed = true;
	public static boolean canSpinFlywheel = false;
	private DcMotorEx flywheel;
	private GoalTracker goal;
	private Drivetrain drivetrain;
	private static double CameraDisplacementX = 0.2;
	private static double CameraHeight = 0.2;
	private static double HeightFromAprilTag = 1;

	public Flywheel(GoalTracker goal, Drivetrain dt) {
		this.goal = goal;
		this.drivetrain = dt;
	}

	private void UpdateDynamicRPS() {
		double distance = goal.getDistance();
		double zRadians = goal.getZRadians();

		double horizontalDistance = CameraDisplacementX + distance * Math.cos(zRadians);
		double verticalDistance = distance * Math.sin(zRadians) + CameraHeight;

		double initialVelocity = desiredFlywheelRPS * Math.PI * FlywheelDiameter / 60;
		double initialVelocityX = Math.cos(ShootingAngle) * initialVelocity;
		double initialVelocityY = Math.sin(ShootingAngle) * initialVelocity;

		double velocityInVacuum =
				Math.sqrt(Gravity * Math.sqrt(verticalDistance - InitialCamHeight) / (2 * Math.pow(Math.cos(ShootingAngle), 2)) * InitialCamHeight + verticalDistance * Math.tan(ShootingAngle) - horizontalDistance);

		double flywheelRadius = FlywheelDiameter / 2;
		double Spin, Geff, BallRPM, Vx, Vy, Px, Py;
		BallRPM = maximumFlywheelRPM;
		Vx = initialVelocityX;
		Vy = initialVelocityY;
		Px = CameraDisplacementX;
		Py = InitialCamHeight;

		double desiredRPS = (velocityInVacuum * 60) / (2 * Math.PI * flywheelRadius) * 60;

		double currTime = 0;
		boolean simulationComplete = false;
		while (simulationComplete) {
			Spin = (2 * Math.PI) / (60) * BallRPM * flywheelRadius;
			Geff = Gravity + RotationalDirection * BallGeneralMass * Math.pow(velocityInVacuum, 2) * (verticalDistance - InitialCamHeight) / Spin;
			BallRPM = BallRPM * Math.pow(Math.E, DecaySpin) * currTime;
			Vx = Vx - AerodynamicDrag * TimeInterval;
			Vy = Vy - AerodynamicDrag * Geff * TimeInterval;
			Px = Px + Vx * TimeInterval;
			Py = Py + Vy * TimeInterval;

			if (Px >= horizontalDistance) {
				panelsTelemetry.addData("Flywheel Simulation", "Ball reached simulated position.");
				break;
			};

			if (currTime > SimulationTimeout) {
				simulationComplete = true;
				if (!(Px >= horizontalDistance)) {
					panelsTelemetry.addData("Flywheel Simulation", "Ball failed to reach simulated position.");
					panelsTelemetry.addData("Flywheel Simulation", "desiredRPS increased by 5% to account for failure.");
					desiredRPS += desiredRPS / 100 * 5;
				};
				break;
			};
			currTime += TimeInterval;
		}

		desiredFlywheelRPS = desiredRPS;
	};

	public static void setDesiredFlywheelRPS(double rps) {
		desiredFlywheelRPS = rps;
	}

	@Override
	public void init(RobotHardwareGroup robotHardware) {
		flywheel = robotHardware.flywheel;
		flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
		flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	@Override
	public void update() {
		if (useDynamicSpeed) UpdateDynamicRPS();

		// apply PIDF every frame so @Configurable changes work
		flywheel.setPIDFCoefficients(
				DcMotor.RunMode.RUN_USING_ENCODER,
				new PIDFCoefficients(Globals.flywheelKP, 0, 0, Globals.flywheelKF));

		flywheel.setVelocity(canSpinFlywheel ? desiredFlywheelRPS * GOBILDA_5203_6000RPM : 0);

		panelsGraph.addData("Flywheel Vel", canSpinFlywheel ? desiredFlywheelRPS * GOBILDA_5203_6000RPM : 0);
		panelsGraph.addData("Flywheel RPS", getFlywheelRPS());
		panelsGraph.addData("Flywheel Error", getFlywheelErrorRPS());
	}

	public double getFlywheelRPS() {
		return flywheel.getVelocity() / GOBILDA_5203_6000RPM;
	}

	public void runFlywheel() {

	}

	public double getFlywheelErrorRPS() {
		return getFlywheelRPS() - desiredFlywheelRPS;
	}

	public boolean isReadyForFeed() {
		return Math.abs(getFlywheelErrorRPS()) <= FLYWHEEL_ERROR_TOL;
	}

}