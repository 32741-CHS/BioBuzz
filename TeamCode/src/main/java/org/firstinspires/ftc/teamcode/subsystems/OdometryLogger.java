package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.Globals.simpleTelemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

@Configurable
public class OdometryLogger implements SubsystemBase {
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void init(RobotHardwareGroup robotHardware) {
        pinpoint = robotHardware.pinpoint;
    }

    @Override
    public void update() {
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();
        simpleTelemetry.addData("Odometry (CM) X", pos.getX(DistanceUnit.INCH));
        simpleTelemetry.addData("Odometry (CM) Y", pos.getY(DistanceUnit.INCH));
        simpleTelemetry.addData("Odometry (DEGREES) Heading", pos.getHeading(AngleUnit.DEGREES));
    }
}
