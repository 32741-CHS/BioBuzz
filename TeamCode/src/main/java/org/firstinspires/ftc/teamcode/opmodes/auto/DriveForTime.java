package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.configs.Globals.robotHardware;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Configurable
@Autonomous(name = "Drive for time", group = "Robot")
public class DriveForTime extends LinearOpMode {
    public static double DRIVE_TIME = 0.55;
    public static double DRIVE_POWER = 0.4;

    public static double WAIT_TIME = 20;
    private final ElapsedTime timer = new ElapsedTime();
    private Drivetrain drivetrain;

    @Override
    public void runOpMode() {
        robotHardware.init(hardwareMap);
        drivetrain = new Drivetrain();

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
