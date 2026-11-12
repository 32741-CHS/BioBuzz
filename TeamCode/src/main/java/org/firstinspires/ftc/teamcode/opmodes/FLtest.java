package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "FLtest")
public class FLtest extends OpMode {
    private DcMotor motor;


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "flDrive");
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
        telemetry.addData("MotorEncoder", motor.getCurrentPosition());
        if (gamepad1.right_trigger_pressed) {
            telemetry.addData("On", 1);
            motor.setPower(1);
        } else {
            telemetry.addData("Of", 1);
            motor.setPower(0);
        }
    }


}