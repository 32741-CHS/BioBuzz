package org.firstinspires.ftc.teamcode.subsystems;

import java.util.HashMap;

interface SubSystemRequirements {
    void update();
    default void stop(){};
}
