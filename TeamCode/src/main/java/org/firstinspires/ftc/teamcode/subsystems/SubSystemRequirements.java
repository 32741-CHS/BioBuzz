package org.firstinspires.ftc.teamcode.subsystems;

import java.util.HashMap;

interface SubSystemRequirements {
    public void init();
    public void update(HashMap<String, Object> frame); // Frame will actuly be a frame object
    public void terminate();
}
