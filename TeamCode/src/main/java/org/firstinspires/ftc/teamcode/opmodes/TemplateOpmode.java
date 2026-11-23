package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.simpleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.configs.BotHardware;

@Autonomous(name = "TemplateOpmode", group = "Tests")
public class TemplateOpmode extends OpMode {
    private BotHardware botHardware;

    @Override
    public void init() {
        simpleTelemetry.registerFtcTelemetry(telemetry);
        botHardware = new BotHardware(hardwareMap);
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
        simpleTelemetry.addData(key, value);
    }


}