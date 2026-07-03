package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp.isRed;
import static org.firstinspires.ftc.teamcode.utils.AprilTags.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.utils.AprilTags.RED_GOAL;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.configs.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Ballistics;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Configurable
@Autonomous(name="Drive and shoot", group="Robot")
public class DriveAndShoot extends LinearOpMode {
    public static double DRIVE_TIME = 1.2;
    public static double DRIVE_POWER = 0.4;

    public static double SHOOT_TIME = 5;
    public static double MAX_SPINUP_TIME = 4;

    public static double FLYWHEEL_ERROR_TOL = 1;


    private final RobotHardware hw = new RobotHardware();

    private Drivetrain drivetrain;
    private Shooter shooter;
    private Intake intake;
    private Vision vision;
    private Turret turret;

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        hw.init(hardwareMap);
        drivetrain = new Drivetrain(hw);
        shooter = new Shooter(hw);
        intake = new Intake(hw);
        turret = new Turret(hw);
        vision = new Vision(hw);

        while (opModeInInit()) {
            if (gamepad1.x) {
                isRed = true;
            } else if (gamepad1.a) {
                isRed = false;
            }

            telemetry.addData("Team", isRed ? "RED" : "BLUE");
            telemetry.addData("Switch", "Square = Red, X = Blue");
            telemetry.update();
        }

        waitForStart();

        Shooter.canSpinFlywheel = true;
        Shooter.desiredFlywheelRPS = 16.5;

        timer.reset();
        while (opModeIsActive() && Math.abs(shooter.getFlywheelErrorRPS()) > FLYWHEEL_ERROR_TOL && timer.seconds() <= MAX_SPINUP_TIME) {
            trackTag();
            turret.update();
            shooter.update();
        }

        timer.reset();
        while (opModeIsActive() && timer.seconds() <= SHOOT_TIME) {
            trackTag();
            shooter.feed();
            intake.eat();
            turret.update();
            shooter.update();
            intake.update();
        }

        timer.reset();

        while (opModeIsActive() && timer.seconds() <= DRIVE_TIME) {
            drivetrain.drive(DRIVE_POWER, 0, 0, false);
        }
    }

    private void trackTag() {
        AprilTagDetection goalTag = vision.getTagById(isRed ? RED_GOAL : BLUE_GOAL);
        if (goalTag != null) {
            double bearing = Math.toDegrees(goalTag.ftcPose.bearing);
            double angle = Ballistics.calculateTurretAngle(bearing, turret.getCurrentAngle());
            turret.goTo(angle);
        }
    }
}