package org.firstinspires.ftc.teamcode.opmodes;
//This file is meant to initialise most of the stuff in one file.
//By the way, this entire codebase is meant to use public variables for state lol
import static org.firstinspires.ftc.teamcode.configs.Globals.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.configs.Globals.RED_GOAL;
import static org.firstinspires.ftc.teamcode.configs.Globals.robotHardwareGroup;
import static org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp.isRed;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Ballistics;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Primary file for handling orchestration of robot commands.
 * This doesn't actually do anything, it just handles sending the signals to the game phases and
 * keeping track of robot state.
 */
class primaryRobot {
    private void startBotInit()  {
        robotHardwareGroup.init(hardwareMap);
        drivetrain = new Drivetrain();
        shooter = new Shooter();
        intake = new Intake();
        turret = new Turret();
        vision = new Vision();

        while (opModeInInit()) { //while initialising opmode.. (this is probably intentionally called multiple times)
            if (gamepad1.x) {
                isRed = true;
            } else if (gamepad1.a) { //check what team we're on
                isRed = false;
            }
            telemetry.addData("Team", isRed ? "RED" : "BLUE"); //set the telemetry to match the detected team
            telemetry.addData("Switch", "Square = Red, X = Blue");
            telemetry.update(); //and update the telemetry reader
        }
    }

}
public enum gamePhase {
    AUTO, TELEOP, ENDGAME;
}