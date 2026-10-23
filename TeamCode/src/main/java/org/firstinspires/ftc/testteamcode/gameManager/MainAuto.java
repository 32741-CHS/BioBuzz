package org.firstinspires.ftc.testteamcode.gameManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Main Autonomous")
public class MainAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // This phase onwards is init.

        waitForStart();
        // Start button is pressed.
        while (opModeIsActive()) { // Game loop for auto.
            //
        }
    }
}
