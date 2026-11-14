package org.firstinspires.ftc.teamcode.opmodes.debug_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest", group = "Tests")
public class ServoTest extends OpMode {
    private Servo servo;
    private DcMotor motor;
    private double pos;


    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
        motor = hardwareMap.get(DcMotor.class, "frDrive");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        telemetry.addData("MotorEnder", motor.getCurrentPosition());
        if (gamepad1.right_trigger_pressed) {
            telemetry.addData("Position", 1);
            servo.setPosition(1);
        } else {
            telemetry.addData("Position", 1);
            servo.setPosition(0);
        }
    }


}