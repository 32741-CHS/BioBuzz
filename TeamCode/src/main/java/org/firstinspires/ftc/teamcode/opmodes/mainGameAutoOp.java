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

@Autonomous(name = "mainGameAutoOp", group = "Autonomous")
public class mainGameAutoOp extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose point1 = poseFactory.of(20.8199, 54.4297, 90.2049);
    private final Pose point1Control1 = poseFactory.of(53.2297, 47.0934, 0);
    private final Pose point1Control2 = poseFactory.of(20.9419, 20.3065, 0);
    private final Pose point2 = poseFactory.of(20.9205, 8.5954, 90.1258);
    private final Pose point3 = poseFactory.of(56.5363, 30.3538, 92.8459);
    private final Pose point3Control1 = poseFactory.of(20.4142, 24.5583, 0);
    private final Pose point3Control2 = poseFactory.of(40.8085, 37.0766, 0);
    private final Pose point3Control3 = poseFactory.of(35.4676, 10.3701, 0);
    private final Pose point3Control4 = poseFactory.of(57.9619, 1.676, 0);

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
        return Paths.curve(start, point1Control1, point1Control2, point1).tangent();
    }

    public Path path2() {
        return Paths.line(point1, point2).reverseTangent();
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3Control2, point3Control3, point3Control4, point3).tangent();
    }
}


