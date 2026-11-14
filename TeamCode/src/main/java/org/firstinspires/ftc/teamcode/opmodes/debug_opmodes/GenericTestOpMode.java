package org.firstinspires.ftc.teamcode.opmodes.debug_opmodes;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.configs.pedro.Constants;

@Autonomous(name = "GenericTestOpMode", group = "Tests")
public class GenericTestOpMode extends OpMode {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(56, 36, 90);
    private final Pose point2 = poseFactory.of(29.8785, 36.1122, 179.7539);
    private final Pose point3 = poseFactory.of(82.4498, 35.8224, 179.6842);
    private final Pose point4 = poseFactory.of(56.0767, 8.6855, -90.4675);
    private final Pose point4Control1 = poseFactory.of(56.3014, 36.2195, 0);
    private final Pose point5 = poseFactory.of(29.9558, 36.0896, 0.482);
    private final Pose point5Control1 = poseFactory.of(56.2407, 36.3107, 0);
    private final Pose point6 = poseFactory.of(56.1905, 9.1712, 90.1096);
    private final Pose point6Control1 = poseFactory.of(56.1392, 35.9608, 0);

    private Follower follower;

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);

        Scheduler.reset();
        Scheduler.schedule(sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6())
        ));
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).tangent();
    }

    public Path path3() {
        return Paths.line(point2, point3).reverseTangent();
    }

    public Path path4() {
        return Paths.curve(point3, point4Control1, point4).tangent();
    }

    public Path path5() {
        return Paths.curve(point4, point5Control1, point5).reverseTangent();
    }

    public Path path6() {
        return Paths.curve(point5, point6Control1, point6).reverseTangent();
    }
}


