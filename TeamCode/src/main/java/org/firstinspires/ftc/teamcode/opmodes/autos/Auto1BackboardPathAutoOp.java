package org.firstinspires.ftc.teamcode.opmodes.autos;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.configs.Globals.FLYWHEEL_POWER;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.configs.pedro.Constants;

@Autonomous(name = "Auto1BackboardPathAutoOp", group = "Pedro")
public class Auto1BackboardPathAutoOp extends LinearOpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(132.0946, 8.7228, 90);
    private final Pose path1 = poseFactory.of(106.3001, 132.7247, 90.4734);
    private final Pose path1Control1 = poseFactory.of(133.8959, 49.1291, 0);
    private final Pose path1Control2 = poseFactory.of(106.5948, 97.0636, 0);
    private Follower follower;

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        RobotHardwareGroup robotHardware = new RobotHardwareGroup(hardwareMap);
        instant(() -> {
            robotHardware.flywheel.setPower(FLYWHEEL_POWER);
        });

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1())
        );
    }

    public Path path1() {
        return Paths.curve(start, path1Control1, path1Control2, path1).tangent();
    }
}
