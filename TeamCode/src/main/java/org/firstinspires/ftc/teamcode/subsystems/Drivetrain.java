package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.configs.TickRates.GOBILDA_5203_312RPM;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.configs.RobotHardware;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Drivetrain {

    static final double WHEEL_DIAMETER = 10.4;
    static final double COUNTS_PER_CM = GOBILDA_5203_312RPM / (WHEEL_DIAMETER * Math.PI);

    private static final double SPEED_SLOW   = 0.4;
    private static final double SPEED_NORMAL = 1;

    private static final double STICK_DEADBAND = 0.05;

    public static double AUTO_DRIVE_SPEED = 0.4;

    private final DcMotor flDrive, frDrive, blDrive, brDrive;
    private final IMU imu;

    private double speedMultiplier = SPEED_NORMAL;

    public Drivetrain(RobotHardware hw) {
        (flDrive = hw.flDrive).setDirection(DcMotorSimple.Direction.REVERSE);
        (blDrive = hw.blDrive).setDirection(DcMotorSimple.Direction.REVERSE);
        (frDrive = hw.frDrive).setDirection(DcMotorSimple.Direction.FORWARD);
        (brDrive = hw.brDrive).setDirection(DcMotorSimple.Direction.FORWARD);
        imu = hw.imu;

        for (DcMotor m : new DcMotor[]{flDrive, frDrive, blDrive, brDrive}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        resetIMU();
    }

		// normalizes speed (meters per second) to a value from 0 - 1
		// makes it viable to use for DcMotor.setPower()
		public float NormalizeSpeed(float speed) {
			float WHEEL_RADIUS = (float)WHEEL_DIAMETER / 2;
			float RPM = 312;
			return 2 * (float)Math.PI * WHEEL_RADIUS / 60 * RPM;
		}
		
		// updates the speed of a motor
		// uses normalizespeed func to normalize it to be 0 - 1
		public void UpdateSpeed(DcMotor drive, float speed) {
			drive.setPower(NormalizeSpeed(speed));
			return;
		}

		// updates the direction of a motor
		public void UpdateDirection(DcMotor drive, DcMotorSimple.Direction direction) {
			drive.setDirection(direction);
			return;
		}

		public void resetIMU() {
			imu.initialize(new IMU.Parameters(
					new RevHubOrientationOnRobot(
						RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
						RevHubOrientationOnRobot.UsbFacingDirection.DOWN
				)
			));
		}

}
