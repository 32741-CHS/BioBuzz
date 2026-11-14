package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configs.pedro.Constants;

@Autonomous(name = "GameAutoOpTest2", group = "Autonomous")
public class GameAutoOpTest2 extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(12.7861, 8.2996, -90);
    private final Pose path1Control1 = poseFactory.of(58.824, 31.9218, 0);
    private final Pose path1Control2 = poseFactory.of(32.2515, 39.6342, 0);
    private final Pose path1Control3 = poseFactory.of(11.3475, 33.6949, 0);
    private final Pose point2 = poseFactory.of(57.5971, 16.003, 91.3625);
    private final Pose point2Control1 = poseFactory.of(12.4485, 34.4132, 0);
    private final Pose point2Control2 = poseFactory.of(56.8479, 47.5044, 0);
    private final Pose point3 = poseFactory.of(57.3088, 27.5634, 91.4289);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

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

    public Path path1() {
        return Paths.curve(start, path1Control1, path1Control2, path1Control3, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2Control2, point2).reverseTangent();
    }

    public Path path3() {
        return Paths.line(point2, point3).tangent();
    }
}
