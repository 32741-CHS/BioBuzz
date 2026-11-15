package org.firstinspires.ftc.teamcode.opmodes.debug_opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;

@Autonomous(name = "OdometerTest", group = "Tests")
public class OdometerTest extends OpMode {

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        log("HOW TO USE", "Move the robot and see the readings");
        panelsTelemetry.update();
        telemetry.update();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, RobotHardwareGroup.PINPOINT_NAME);
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();
    }


    @Override
    public void loop() {
        Pose2D pos = pinpoint.getPosition();

        log("X INCH", pos.getX(DistanceUnit.INCH));
        log("Y INCH", pos.getY(DistanceUnit.INCH));
        log("Heading RADIANS", pos.getHeading(AngleUnit.RADIANS));
        panelsTelemetry.update();
        telemetry.update();

    }

    private void log(String key, Object value) {
        panelsTelemetry.addData(key, value);
        telemetry.addData(key, value);
    }


}