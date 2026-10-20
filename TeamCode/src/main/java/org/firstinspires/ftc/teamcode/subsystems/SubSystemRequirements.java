package org.firstinspires.ftc.teamcode.subsystems;

import java.util.HashMap;

interface SubSystemRequirements {
    default void init() {}

    default void update() {} // Frame will actuly be a frame object

    default void terminate() {}
}
