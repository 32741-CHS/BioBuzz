package org.firstinspires.ftc.teamcode.opmodes.debug_opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.GOBILDA_5203_312RPM;
import static org.firstinspires.ftc.teamcode.configs.Globals.GOBILDA_5203_6000RPM;
import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Configurable
@Autonomous(name = "MotorTest", group = "Tests")
public class MotorTest extends OpMode {
    public static String motorName = "flDrive";
    public static double requestedPower = 1;

    @Override
    public void init() {
        log("HOW TO USE", "Controller/Gamepad 1 right stick changes requestedPower");
        panelsTelemetry.update();
        telemetry.update();
    }


    @Override
    public void loop() {
        DcMotorEx motor = (DcMotorEx) hardwareMap.get(DcMotor.class, motorName);

        /// If Panels doesn't work
        requestedPower = Math.clamp(requestedPower - gamepad1.left_stick_y / 10, -1, 1);

        motor.setPower(requestedPower);
        log("Motor Name", motorName);
        log("Requested Power", requestedPower);
        log("Approx Velocity (6000 RMP motor)", motor.getVelocity() * GOBILDA_5203_6000RPM);
        log("Approx Velocity (312 RMP motor)", motor.getVelocity() * GOBILDA_5203_312RPM);
        panelsTelemetry.update();
        telemetry.update();

    }

    private void log(String key, Object value) {
        panelsTelemetry.addData(key, value);
        telemetry.addData(key, value);
    }


}