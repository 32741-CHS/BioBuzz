package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "BioBuzz TeleOp")
public class MainTeleOp extends OpMode {
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // EXAMPLE starting position. Replace with your actual position.
    private final Pose startPose = poseFactory.of(24, 24, 0);
    Gamepad driverGamepad, shooterGamepad;
    private Follower follower;

    @Override
    public void init() {

        driverGamepad = gamepad1;
        shooterGamepad = gamepad2;
        Scheduler.reset(); // Init Ivy
        follower = Constants.create(hardwareMap); // Init Pedro
        follower.setPose(startPose);

        // Our Subsystems

    }

    @Override
    public void loop() {

        // Manual driving
        follower.manual(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        // Update Pedro localisation and drivetrain
        follower.update();

        // Update Ivy commands
        Scheduler.execute();

        // Retrieve current robot position
        Pose pose = follower.pose();

        // Telemetry
        telemetry.addData("X", "%.2f", pose.x());
        telemetry.addData("Y", "%.2f", pose.y());

        telemetry.addData(
                "Heading",
                "%.2f°",
                Math.toDegrees(pose.heading())
        );

        telemetry.addData("Pedro Mode", follower.mode());

        telemetry.update();
    }
}