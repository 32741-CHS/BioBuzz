package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.Globals.AerodynamicDrag;
import static org.firstinspires.ftc.teamcode.configs.Globals.BallGeneralMass;
import static org.firstinspires.ftc.teamcode.configs.Globals.DecaySpin;
import static org.firstinspires.ftc.teamcode.configs.Globals.FLYWHEEL_ERROR_TOL;
import static org.firstinspires.ftc.teamcode.configs.Globals.FlywheelDiameter;
import static org.firstinspires.ftc.teamcode.configs.Globals.GOBILDA_5203_6000RPM;
import static org.firstinspires.ftc.teamcode.configs.Globals.Gravity;
import static org.firstinspires.ftc.teamcode.configs.Globals.InitialCamHeight;
import static org.firstinspires.ftc.teamcode.configs.Globals.NectarMass;
import static org.firstinspires.ftc.teamcode.configs.Globals.PollenMass;
import static org.firstinspires.ftc.teamcode.configs.Globals.RotationalDirection;
import static org.firstinspires.ftc.teamcode.configs.Globals.ShootingAngle;
import static org.firstinspires.ftc.teamcode.configs.Globals.SimulationTimeout;
import static org.firstinspires.ftc.teamcode.configs.Globals.SlipagePercentage;
import static org.firstinspires.ftc.teamcode.configs.Globals.TimeInterval;
import static org.firstinspires.ftc.teamcode.configs.Globals.panelsGraph;
import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.configs.Globals;
import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.BallType;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

@Configurable
public class Flywheel implements SubsystemBase {
	public static double desiredFlywheelRPS = 13;
	public static double maximumFlywheelRPM = 6000;
	public static double minimumFlywheelRPM = 0;
	public static boolean useDynamicSpeed = true;
	public static boolean canSpinFlywheel = false;
	private DcMotorEx flywheel;
	private Camera camera;
	private Drivetrain drivetrain;
	private WebcamName insideCam;
	private static double CameraDisplacementX = 0.06; // made up number, change when cam added
	private static double CameraHeight = 0.37; // made up number, change when cam added
	private static double HeightFromAprilTag = 0.15; // tweak if not good
	// NEW: lumped Magnus (spin lift) constant, units 1/m. 0 = Magnus off. Tune on the field.
	private static double MagnusCoefficient = 0.019;

	// goal stuff
	private double distance = Double.NaN;
	private double zRadians = Double.NaN;

	public Flywheel(Drivetrain dt, Camera cam) {
		this.drivetrain = dt;
		this.camera = cam;
	}

	public void changeGoal(double distance, double zRadians) {
		this.distance = distance;
		this.zRadians = zRadians;
	}

	private void UpdateDynamicRPS() {
		if (Double.isNaN(distance) || Double.isNaN(zRadians)) return;

		double horizontalDistance = CameraDisplacementX + distance * Math.cos(zRadians);
		double verticalDistance = distance * Math.sin(zRadians) + CameraHeight;

		// MATH FIX 1: standard projectile equation, ball speed needed with NO air resistance
		// v^2 = g x^2 / (2 cos^2(theta) (x tan(theta) - dy))
		double heightDiff = verticalDistance - InitialCamHeight;
		double denominator = 2 * Math.pow(Math.cos(ShootingAngle), 2)
				* (horizontalDistance * Math.tan(ShootingAngle) - heightDiff);
		if (denominator <= 0) return; // target can't be hit at this launch angle (only added line that isn't maths)
		double velocityInVacuum = Math.sqrt(Gravity * horizontalDistance * horizontalDistance / denominator);

		double flywheelRadius = FlywheelDiameter / 2;

		// MATH FIX 2: ball leaves slower than the wheel surface, so DIVIDE by (1 - slip).
		// MATH FIX 3: RPS = surface speed / circumference (no *60 factors).
		double desiredRPS = (velocityInVacuum / (1 - SlipagePercentage)) / (Math.PI * FlywheelDiameter);

		// MATH FIX 4: simulation starts from the speed we just calculated, not the old desiredFlywheelRPS
		double initialVelocityX = Math.cos(ShootingAngle) * velocityInVacuum;
		double initialVelocityY = Math.sin(ShootingAngle) * velocityInVacuum;

		double Spin, Geff, BallRPM, Vx, Vy, Px, Py;
		BallRPM = desiredRPS * 60; // MATH FIX 5: spin comes from the wheel speed we are asking for
		Vx = initialVelocityX;
		Vy = initialVelocityY;
		Px = 0; // MATH FIX 6: horizontalDistance is already measured from the launch point
		Py = InitialCamHeight;

		double ballMass = BallGeneralMass;
		if (camera.getBallType(insideCam) == BallType.Big) {
			ballMass = NectarMass;
		} else if (camera.getBallType(insideCam) == BallType.Small) {
			ballMass = PollenMass;
		}

		// AerodynamicDrag is now k in a = -k|v|v (units 1/m, = 0.5*rho*Cd*A/mass), scaled for ball mass
		double dragK = AerodynamicDrag * BallGeneralMass / ballMass;

		double currTime = 0;
		boolean simulationComplete = false;
		while (!simulationComplete) {
			double speed = Math.hypot(Vx, Vy);
			Spin = (2 * Math.PI) / (60) * BallRPM * flywheelRadius;
			// MATH FIX 7: Magnus lift grows WITH spin and speed (was divided by spin). Vertical-only approximation.
			// RotationalDirection: -1 = backspin (lifts the ball), +1 = topspin.
			Geff = Gravity + RotationalDirection * MagnusCoefficient * Spin * speed;
			// MATH FIX 8: exponential decay per step (was multiplying by currTime, which zeroed it)
			BallRPM = BallRPM * Math.exp(-DecaySpin * TimeInterval);
			// MATH FIX 9: drag is proportional to speed^2, and gravity acts at full strength on Vy
			Vx = Vx - dragK * speed * Vx * TimeInterval;
			Vy = Vy - (Geff + dragK * speed * Vy) * TimeInterval;
			Px = Px + Vx * TimeInterval;
			Py = Py + Vy * TimeInterval;

			if (Py <= 0.001) {
				panelsTelemetry.addData("Flywheel Simulation", "Ball altitude cannot be less than zero.");
				desiredRPS += desiredRPS / 100 * 5;
				break;
			}

			// MATH FIX 10: "reached" now means crossing the goal distance AT or ABOVE the goal height
			if (Px >= horizontalDistance) {
				if (Py >= verticalDistance) {
					panelsTelemetry.addData("Flywheel Simulation", "Ball reached simulated position.");
				} else {
					panelsTelemetry.addData("Flywheel Simulation", "Ball arrived below target, desiredRPS increased by 5%.");
					desiredRPS += desiredRPS / 100 * 5;
				}
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
		this.insideCam = robotHardware.insideCam;
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