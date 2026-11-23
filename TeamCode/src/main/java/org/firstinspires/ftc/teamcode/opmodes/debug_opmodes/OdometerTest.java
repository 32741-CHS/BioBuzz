package org.firstinspires.ftc.teamcode.opmodes.debug_opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.simpleTelemetry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.configs.BotHardware;

@Autonomous(name = "OdometerTest", group = "Tests")
public class OdometerTest extends OpMode {

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        simpleTelemetry.registerFtcTelemetry(telemetry);
        simpleTelemetry.addData("HOW TO USE", "Move the robot and see the readings");
        simpleTelemetry.update();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, BotHardware.PINPOINT_NAME);
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();
    }


    @Override
    public void loop() {
        Pose2D pos = pinpoint.getPosition();

        simpleTelemetry.addData("X INCH", pos.getX(DistanceUnit.INCH));
        simpleTelemetry.addData("Y INCH", pos.getY(DistanceUnit.INCH));
        simpleTelemetry.addData("Heading RADIANS", pos.getHeading(AngleUnit.RADIANS));
        simpleTelemetry.update();

    }


}