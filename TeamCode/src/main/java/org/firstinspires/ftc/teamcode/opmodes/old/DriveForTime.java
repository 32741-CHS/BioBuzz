package org.firstinspires.ftc.teamcode.opmodes.old;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.subsystems.old.OldDrivetrain;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

@Disabled
@Configurable
@Autonomous(name = "Drive for time", group = "Robot")
public class DriveForTime extends LinearOpMode {
    public static final double DRIVE_TIME = 0.55;
    public static final double DRIVE_POWER = 0.4;

    public static final double WAIT_TIME = 20;
    private final ElapsedTime timer = new ElapsedTime();
    private RobotHardwareGroup robotHardware;
    private OldDrivetrain oldDrivetrain;

    @Override
    public void runOpMode() {
        // New robotHardware To make force fresh init and reduce risk of improper motor
        // config.
        robotHardware = new RobotHardwareGroup();
        robotHardware.init(hardwareMap);
        (oldDrivetrain = new OldDrivetrain()).init(robotHardware);

        waitForStart();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < WAIT_TIME) {
        }
        timer.reset();

        while (opModeIsActive() && timer.seconds() < DRIVE_TIME) {
            oldDrivetrain.drive(new Vector2D(DRIVE_POWER, 0), 0, false);
        }

        oldDrivetrain.stop();
    }
}
