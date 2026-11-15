package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.simpleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;

@Autonomous(name = "TemplateOpmode", group = "Tests")
public class TemplateOpmode extends OpMode {
    private RobotHardwareGroup robotHardwareGroup;

    @Override
    public void init() {
        robotHardwareGroup = new RobotHardwareGroup(hardwareMap);
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
        log(key, value);
    }

    private void log(String key, double value) {
        simpleTelemetry.addData(key, value);
        telemetry.addData(key, value);
    }

    private void log(String key, Object value) {
        simpleTelemetry.addData(key, value);
        telemetry.addData(key, value);
    }

}