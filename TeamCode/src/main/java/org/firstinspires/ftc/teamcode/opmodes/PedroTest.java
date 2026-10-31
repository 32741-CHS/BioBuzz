package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;



public class PedroTest {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(25.1375, 44.1099, 117.7149);
    private final Pose path1Control1 = poseFactory.of(1.8706, 3.9841, 0);
    private final Pose path1Control2 = poseFactory.of(30.077, 34.7075, 0);
    private final Pose point2 = poseFactory.of(56.9423, 127.0139, 37.4868);
    private final Pose point2Control1 = poseFactory.of(10.3225, 91.2583, 0);

    public Path path1() {
        return Paths.curve(start, path1Control1, path1Control2, path1).tangent();
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2).tangent();
    }
}
