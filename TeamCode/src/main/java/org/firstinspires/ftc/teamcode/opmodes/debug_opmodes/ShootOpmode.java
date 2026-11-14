package org.firstinspires.ftc.teamcode.opmodes.debug_opmodes;

import static java.lang.Math.round;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "ShootOpmode", group = "Tests")
public class ShootOpmode extends LinearOpMode {

    @Override
    public void runOpMode() {
        // New robotHardware To make force fresh init and reduce risk of improper motor
        // config.
        while (opModeInInit()) {

            telemetry.addData("state", "init");
            telemetry.update();
        }

        waitForStart();


        DcMotor feeder = hardwareMap.get(DcMotor.class, "feeder");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor shooter = hardwareMap.get(DcMotor.class, "flywheel");
        Servo servo = hardwareMap.get(Servo.class, "servo");

        feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (opModeIsActive()) {
            feeder.setPower((gamepad1.left_stick_y + 1) / 2);
            intake.setPower((gamepad1.left_stick_y + 1) / 2);
            shooter.setPower((gamepad1.left_stick_y + 1) / 2);
            servo.setPosition(round(time / 10) % 2 == 0 ? Servo.MIN_POSITION : Servo.MAX_POSITION);
            telemetry.addData("Running", true);
        }
    }

}
