package org.firstinspires.ftc.teamcode.opmodes.old;

import static org.firstinspires.ftc.teamcode.configs.Globals.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.configs.Globals.RED_GOAL;
import static org.firstinspires.ftc.teamcode.opmodes.old.OldMainTeleOp.isRed;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.subsystems.old.OldDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.old.OldIntake;
import org.firstinspires.ftc.teamcode.subsystems.old.OldShooter;
import org.firstinspires.ftc.teamcode.subsystems.old.OldTurret;
import org.firstinspires.ftc.teamcode.utils.Ballistics;
import org.firstinspires.ftc.teamcode.utils.Vector2D;
import org.firstinspires.ftc.teamcode.utils.old.OldVision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Disabled
@Configurable
@Autonomous(name = "Drive and shoot close", group = "Robot")
public class DriveAndShootClose extends LinearOpMode {
    public static final double DRIVE_TIME = 2;
    public static final double DRIVE_POWER = 0.4;

    public static final double SHOOT_TIME = 10;
    public static final double MAX_SPINUP_TIME = 5;

    public static final double FLYWHEEL_ERROR_TOL = 100;
    private final ElapsedTime timer = new ElapsedTime();
    private OldDrivetrain oldDrivetrain;
    private OldShooter oldShooter;
    private OldIntake oldIntake;
    private OldVision oldVision;
    private OldTurret oldTurret;
    private double lastGoalTagTime = 0;
    private RobotHardwareGroup robotHardware;

    @Override
    public void runOpMode() {
        // New robotHardware To make force fresh init and reduce risk of improper motor
        // config.
        robotHardware = new RobotHardwareGroup();
        robotHardware.init(hardwareMap);
        (oldDrivetrain = new OldDrivetrain()).init(robotHardware);
        (oldShooter = new OldShooter()).init(robotHardware);
        (oldIntake = new OldIntake()).init(robotHardware);
        (oldTurret = new OldTurret()).init(robotHardware);
        (oldVision = new OldVision()).init(robotHardware);

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

        OldShooter.canSpinFlywheel = true;
        lastGoalTagTime = getRuntime();

        timer.reset();
        while (opModeIsActive() && timer.seconds() <= DRIVE_TIME) {
            oldDrivetrain.drive(new Vector2D(-DRIVE_POWER, 0), 0, false);
        }

        timer.reset();
        oldDrivetrain.stop();

        while (opModeIsActive()
                && Math.abs(oldShooter.getFlywheelErrorRPS()) > FLYWHEEL_ERROR_TOL
                && timer.seconds() <= MAX_SPINUP_TIME) {
            runBallistics();
            oldTurret.update();
            oldShooter.update();
        }

        timer.reset();
        while (opModeIsActive() && timer.seconds() <= SHOOT_TIME) {
            runBallistics();
            oldShooter.feed();
            oldIntake.eat();
            oldTurret.update();
            oldShooter.update();
            oldIntake.update();
        }

        timer.reset();
        while (opModeIsActive() && timer.seconds() <= DRIVE_TIME) {
            oldDrivetrain.drive(new Vector2D(0, DRIVE_POWER), 0, false);
        }

        oldDrivetrain.stop();

        timer.reset();
    }

    private void runBallistics() {
        AprilTagDetection goalTag = oldVision.getTagById(isRed ? RED_GOAL : BLUE_GOAL);
        if (goalTag != null) {
            lastGoalTagTime = getRuntime();
            double bearing = Math.toDegrees(goalTag.ftcPose.bearing);
            double angle = Ballistics.calculateTurretAngle(bearing, oldTurret.getCurrentAngle());
            oldTurret.goTo(angle);

            double distance = goalTag.ftcPose.range;
            OldShooter.setDesiredFlywheelRPS(Ballistics.calculateFlywheelRPS(distance));
        } else if (getRuntime() - lastGoalTagTime > OldTurret.LOST_TAG_RETURN_DELAY) {
            oldTurret.returnHome();
        }
    }
}
