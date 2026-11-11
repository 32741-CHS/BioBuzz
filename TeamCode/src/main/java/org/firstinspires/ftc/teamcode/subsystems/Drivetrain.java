package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.TunableConstants.fieldCentricDriving;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.configs.Globals;
import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.configs.pedro.Constants;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

/// OldDrivetrain class design for movement handling.
/// It controls all four vector wheels + gathers data from odometry.
///
/// ## Functions of value:
/// 1. bindToGamepad (Actives TeleOp Binds)
/// 2. setPosition (Resets internal position tracker)
/// 2. getX
/// 3. getY
/// 4. getHeading
public class Drivetrain implements SubsystemBase {

    private final Pose startPose;
    private GamepadEx driverGamepad;
    private Follower follower;
    private double forward = 0, strafe = 0, turn = 0, powerMultiplier = 1;

    public Drivetrain(Pose startPose) {
        this.startPose = startPose;
    }

    public Drivetrain(double x, double y, double heading) {
        this.startPose = createPose(x, y, heading);
    }

    private static Pose createPose(double x, double y, double heading) {
        return new Pose(x, y, heading);
    }

    public void bindToGamepad(GamepadEx gamepad) {
        driverGamepad = gamepad;
    }

    @Override
    public void init(RobotHardwareGroup robotHardware) {
        follower = Constants.create(robotHardware.hardwareMap);
        this.setPosition(startPose);
        follower.update();
    }

    @Override
    public void update() {
        if (driverGamepad != null) { // Runs only if bindToGamepad called
            forwardInput(powerMultiplier * deadband(-driverGamepad.raw.left_stick_y));
            strafeInput(powerMultiplier * deadband(-driverGamepad.raw.left_stick_x));
            turnInput(powerMultiplier * deadband(driverGamepad.raw.right_stick_x));
        }

        if (fieldCentricDriving) {
            // If the forwards = the same direction relative to the field
            follower.manual(ManualDrive.fieldCentric(forward, strafe, turn, follower.pose().heading()));
        } else {
            // If the forwards = the same direction relative to the robot
            follower.manual(forward, strafe, turn);
        }

        turn = strafe = forward = 0;
        follower.update();
    }

    public void forwardInput(double power) {
        forward = clamp(forward + power);
    }

    private double deadband(double value) {
        return Math.abs(value) < Globals.STICK_DEADBAND ? 0 : value;
    }

    public void strafeInput(double power) {
        strafe = clamp(strafe + power);
    }

    public void turnInput(double power) {
        turn = clamp(turn + power);
    }

    private static double clamp(double value) {
        return Math.max(-1, Math.min(1, value));
    }

    public void speed(double powerMultiplier) {
        this.powerMultiplier = powerMultiplier;
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

    /**
     * X = Front and back on the pitch
     */
    public double getX() {
        return follower.pose().x();
    }

    /**
     * Y = Left and right on the pitch
     */
    public double getY() {
        return follower.pose().y();
    }

    public double getHeading() {
        return follower.pose().heading();
    }

    public Follower getFollower() {
        return follower;
    }
}
