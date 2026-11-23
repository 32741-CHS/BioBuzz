package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.configs.BotHardware;

public interface SubsystemBase {
    void init(BotHardware botHardware);

    void update();
}
