package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.configs.Globals;
import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

public class Drivetrain implements SubsystemBase {

    private final Gamepad gamepad;
    private final Pose startPose;
    private Follower follower;

    public Drivetrain(Gamepad gamepad, Pose startPose) {
        this.startPose = startPose;
        this.gamepad = gamepad;
    }

    public static Pose createPose(double x, double y, double heading) {
        return new Pose(x, y, heading);
    }

    @Override
    public void init(RobotHardwareGroup robotHardware) {
        follower = Constants.create(robotHardware.hardwareMap);
        this.setPosition(startPose);
        follower.update();
    }

    @Override
    public void update() {
        double forward = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = gamepad.right_stick_x;

        forward = deadband(forward);
        strafe = deadband(strafe);
        turn = deadband(turn);

        DrivePowers powers = ManualDrive.fieldCentric(
                forward,
                strafe,
                turn,
                follower.pose().heading()
        );

        follower.manual(powers);
        follower.update();
    }

    public Pose getPosition() {
        return follower.pose();
    }

    public void setPosition(Pose pose) {
        follower.setPose(pose);
    }

    public void setPosition(double x, double y, double heading) {
        follower.setPose(createPose(x, y, heading));
    }


    public double getX() {
        return follower.pose().x();
    }

    public double getY() {
        return follower.pose().y();
    }

    public double getHeading() {
        return follower.pose().heading();
    }

    public Follower getFollower() {
        return follower;
    }

    private double deadband(double value) {
        return Math.abs(value) < Globals.STICK_DEADBAND ? 0 : value;
    }
}
