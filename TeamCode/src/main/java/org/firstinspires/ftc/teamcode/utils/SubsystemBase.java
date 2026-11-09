package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;

public interface SubsystemBase {
    void init(RobotHardwareGroup robotHardware);

    void update();
}
