package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.configs.Globals;
import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class SubsystemHandler implements SubsystemBase {
    private final SubsystemBase[] subsystems;

    public SubsystemHandler(SubsystemBase... subsystems) {
        this.subsystems = subsystems;
    }

    @Override
    public void init(RobotHardwareGroup robotHardware) {
        Scheduler.reset();
        forEverySubsystem(subsys -> subsys.init(robotHardware));
    }

    @Override
    public void update() {
        Scheduler.execute();
        forEverySubsystem(SubsystemBase::update);
        Globals.panelsTelemetry.update();
        panelsTelemetry.update();
    }

    private void forEverySubsystem(Consumer<SubsystemBase> func) {
        for (SubsystemBase subsys : subsystems) {
            func.accept(subsys);
        }
    }

    public void schedule(Command command) {
        Scheduler.schedule(command);
    }

    public void schedule(Runnable func) {
        Scheduler.schedule(instant(func));

    }

    public void schedule(Runnable start, Runnable loop, BooleanSupplier done) {
        Scheduler.schedule(
                Command.build()
                        .setStart(start)
                        .setExecute(loop)
                        .setDone(done)
        );
    }
}
