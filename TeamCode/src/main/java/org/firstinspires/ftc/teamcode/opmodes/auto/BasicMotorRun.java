package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Basic Motor Run", group = "Robot")
public class BasicMotorRun extends LinearOpMode {

    @Override
    public void runOpMode() {
        // 1. Initialize the motor from the hardware map (name must match the Driver Hub config)
        DcMotor motor = hardwareMap.get(DcMotor.class, "motor");

        // 2. Set the motor direction (FORWARD or REVERSE)
        motor.setDirection(DcMotor.Direction.REVERSE);

        // 3. Set run mode (RUN_WITHOUT_ENCODER for basic power control)
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run while the OpMode is active
        while (opModeIsActive()) {
            // Set motor power to 50% speed (-1.0 to 1.0 range
            motor.setPower(0.5);

            telemetry.update();
        }
    }
}
