package org.firstinspires.ftc.teamcode.opmodes.debug_opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.GOBILDA_5203_312RPM;
import static org.firstinspires.ftc.teamcode.configs.Globals.GOBILDA_5203_6000RPM;
import static org.firstinspires.ftc.teamcode.configs.Globals.simpleTelemetry;

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
        simpleTelemetry.registerFtcTelemetry(telemetry);
        simpleTelemetry.addData("HOW TO USE", "Controller/Gamepad 1 right stick changes requestedPower");
        simpleTelemetry.update();
    }


    @Override
    public void loop() {
        DcMotorEx motor = (DcMotorEx) hardwareMap.get(DcMotor.class, motorName);

        /// If Panels doesn't work
        requestedPower = Math.clamp(requestedPower - gamepad1.left_stick_y / 10, -1, 1);

        motor.setPower(requestedPower);
        simpleTelemetry.addData("Motor Name", motorName);
        simpleTelemetry.addData("Requested Power", requestedPower);
        simpleTelemetry.addData("Approx Velocity (6000 RMP motor)", motor.getVelocity() * GOBILDA_5203_6000RPM);
        simpleTelemetry.addData("Approx Velocity (312 RMP motor)", motor.getVelocity() * GOBILDA_5203_312RPM);
        simpleTelemetry.update();

    }


}