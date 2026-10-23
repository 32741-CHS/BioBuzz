package org.firstinspires.ftc.testteamcode.gameManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.testteamcode.utils.gameStates;

// Everything below here was initially meant to be an ultra-manager thingy; realised FTC wants class
// separation.
@Autonomous(name = "Main Robot")
public class MainRobot extends LinearOpMode {
    public static gameStates gameState = gameStates.Inactive;

    @Override
    public void runOpMode()
            throws InterruptedException { // This function runs when INIT is pressed on the DS.
        // initalisation of the ENTIRE bot (hopefully)
        // initialiseRobot();
        // waits for the driver station to press start
        waitForStart();
        gameState = gameStates.Autonomous;
        while (opModeIsActive()) {
            //gameLoop(); // Primary game loop.
            idle(); // Makes sure that other threads can even run; gives time for them to
        }
    }
}
