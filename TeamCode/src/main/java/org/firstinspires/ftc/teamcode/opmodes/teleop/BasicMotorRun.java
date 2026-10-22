package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Basic Motor Run", group = "TeleOp")
public class BasicMotorRun extends LinearOpMode {
    private DcMotor motor;

    @Override
    public void runOpMode() {
        // Map the motor using the name configured in the Driver Hub / Control Hub
        motor = hardwareMap.get(DcMotor.class, "motor");

        // Set the motor run mode (optional, RUN_WITHOUT_ENCODER is default for basic power)
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Run motor at half power (range is -1.0 to 1.0)
            if (gamepad1.rightTriggerWasPressed()) {
                motor.setPower(0.5);
            } else {
                motor.setPower(0);
            }
        }
    }
}
