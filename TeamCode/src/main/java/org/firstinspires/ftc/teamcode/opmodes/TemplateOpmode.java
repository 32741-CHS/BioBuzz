package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "EncoderOdometry Logger", group = "Tests")
public class TemplateOpmode extends OpMode {
    private DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "frDrive");
    }

    @Override //Optional
    public void init_loop() {

    }

    @Override //Optional
    public void start() {
    }

    @Override
    public void loop() {
        String key = "Thing";
        double value = 1;
        panelsTelemetry.addData(key, value);
        telemetry.addData(key, value);
    }


}