package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.configs.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Configurable
@Autonomous(name="Drive for time", group="Robot")
public class DriveForTime extends LinearOpMode {
    public static double DRIVE_TIME = 0.55;
    public static double DRIVE_POWER = 0.4;

    public static double WAIT_TIME = 20;

    private final RobotHardware hw = new RobotHardware();

    private Drivetrain drivetrain;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        hw.init(hardwareMap);
        drivetrain = new Drivetrain(hw);

        waitForStart();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < WAIT_TIME) {}
        timer.reset();

        while (opModeIsActive() && timer.seconds() < DRIVE_TIME) {
            drivetrain.drive(DRIVE_POWER, 0, 0, false);
        }

        drivetrain.stop();
    }
}
