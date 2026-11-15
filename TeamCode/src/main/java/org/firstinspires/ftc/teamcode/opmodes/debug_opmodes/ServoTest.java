package org.firstinspires.ftc.teamcode.opmodes.debug_opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@Autonomous(name = "ServoTest", group = "Tests")
public class ServoTest extends OpMode {
    public static String servoName = "servo";
    public static double requestedPos = 1;

    @Override
    public void init() {
        log("HOW TO USE", "Controller/Gamepad 1 right stick changes requestedPower");
        panelsTelemetry.update();
        telemetry.update();
    }


    @Override
    public void loop() {
        Servo servo = hardwareMap.get(Servo.class, servoName);

        /// If Panels doesn't work
        requestedPos = Math.clamp(requestedPos - gamepad1.left_stick_y / 10, -1, 1);

        servo.setPosition(requestedPos);
        log("Servo Name", servoName);
        log("Requested Position", requestedPos);
        panelsTelemetry.update();
        telemetry.update();

    }

    private void log(String key, Object value) {
        panelsTelemetry.addData(key, value);
        telemetry.addData(key, value);
    }


}