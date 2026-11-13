package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "FLtest", group = "Tests")
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

            panelsTelemetry.addData("State", 1);
            telemetry.addData("State", 1);
            motor.setPower(1);
        } else {
            panelsTelemetry.addData("State", 0);
            telemetry.addData("Of", 0);
            motor.setPower(0);
        }
    }


}