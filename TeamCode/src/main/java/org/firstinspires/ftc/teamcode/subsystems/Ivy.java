package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

import java.util.function.BooleanSupplier;

public class Ivy implements SubsystemBase {


    @Override
    public void init(RobotHardwareGroup robotHardware) {
        Scheduler.reset();
    }

    @Override
    public void update() {
        Scheduler.execute();
    }
}