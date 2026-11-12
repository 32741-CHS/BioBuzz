package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.old.OldMainTeleOp.isRed;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name = "ShootOpmode", group = "Robot")
public class ShootOpmode extends LinearOpMode {

    @Override
    public void runOpMode() {
        // New robotHardware To make force fresh init and reduce risk of improper motor
        // config.
        while (opModeInInit()) {

            telemetry.addData("Team", isRed ? "RED" : "BLUE");
            telemetry.addData("Switch", "Square = Red, X = Blue");
            telemetry.update();
        }

        waitForStart();


        DcMotor feeder = hardwareMap.get(DcMotor.class, "feeder");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor shooter = hardwareMap.get(DcMotor.class, "flywheel");
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (opModeIsActive()) {
            feeder.setPower((gamepad1.left_stick_y + 1) / 2);
            intake.setPower((gamepad1.left_stick_y + 1) / 2);
            shooter.setPower((gamepad1.left_stick_y + 1) / 2);
            telemetry.addData("Running", true);
        }
    }

}
