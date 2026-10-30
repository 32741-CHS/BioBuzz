package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.configs.Globals.TRIGGER_THRESHOLD;

public class GamepadTrigger extends GamepadButton {
    private float percentPressed = 0;

    public float getPercentPressed() {
        return percentPressed;
    }

    public void update(float currentPercentPressed) {
        percentPressed = currentPercentPressed;
        super.update(percentPressed >= TRIGGER_THRESHOLD);
    }
}
