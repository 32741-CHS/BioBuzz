package org.firstinspires.ftc.teamcode.configs.pedro;

// http://192.168.43.1:10158

// pinpoint is plugged into an I2C port other than port 0

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.configs.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.configs.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.configs.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.configs.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.configs.pedro.procedures.TwoWheelTuner;

public class Tuning {
    // Tuners go here
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    /// UnComment when the constants file is set up.

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner((hardwareMap) -> new TwoWheelLocalizer(hardwareMap, Constants.localizerConfig), (hardwareMap) -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }

    @Tuner
    public static Procedure twoWheelTuner() {
        return new TwoWheelTuner();
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), hardwareMap -> new TwoWheelLocalizer(hardwareMap, Constants.localizerConfig), null);
    }

}
