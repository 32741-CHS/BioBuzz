package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "EncoderOdometry Logger")
public class EncoderOdometry extends OpMode {
    private DcMotor xMotor, yMotor;

    @Override
    public void init() {
        xMotor = hardwareMap.get(DcMotor.class, "frDrive");
        xMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        yMotor = hardwareMap.get(DcMotor.class, "flDrive");
        yMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        Object xMotorEncoder = xMotor.getCurrentPosition();
        Object yMotorEncoder = xMotor.getCurrentPosition();

        panelsTelemetry.addData("xMotor", xMotorEncoder);
        telemetry.addData("xMotor", xMotorEncoder);
        panelsTelemetry.addData("yMotor", yMotorEncoder);
        telemetry.addData("yMotor", yMotorEncoder);


    }


}