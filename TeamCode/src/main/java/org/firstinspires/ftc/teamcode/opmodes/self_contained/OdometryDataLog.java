package org.firstinspires.ftc.teamcode.opmodes.self_contained;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "Odometry", group = "Auto")
public class OdometryDataLog extends LinearOpMode {

    private final DcMotor motor = null;

    @Override
    public void runOpMode() {
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();

        while (opModeIsActive()) {
            // Set motor power to 50% speed (-1.0 to 1.0 range)
            pinpoint.update();
            Pose2D pos = pinpoint.getPosition();

            telemetry.addData("X", pos.getX(DistanceUnit.CM));
            telemetry.addData("Y", pos.getY(DistanceUnit.CM));
            telemetry.addData("Heading", pos.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
