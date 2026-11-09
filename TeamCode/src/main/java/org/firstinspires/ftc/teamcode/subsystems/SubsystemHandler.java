package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

import java.util.function.Consumer;

public class SubsystemHandler implements SubsystemBase {
    private final SubsystemBase[] subsystems;

    public SubsystemHandler(SubsystemBase... subsystems) {
        this.subsystems = subsystems;
    }

    public void update() {
        forEverySubsys(SubsystemBase::update);
    }

    private void forEverySubsys(Consumer<SubsystemBase> func) {
        for (SubsystemBase subsys : subsystems) {
            func.accept(subsys);
        }
    }

    public void init(RobotHardwareGroup robotHardware) {
        forEverySubsys(subsys -> subsys.init(robotHardware));
    }


}
