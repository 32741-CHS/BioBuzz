package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.GamepadButton;
import org.firstinspires.ftc.teamcode.utils.GamepadTrigger;

public class GamepadEx {

    public final GamepadButton btnA = new GamepadButton();
    public final GamepadButton btnB = new GamepadButton();
    public final GamepadButton btnX = new GamepadButton();
    public final GamepadButton btnY = new GamepadButton();
    public final GamepadButton leftBumper = new GamepadButton();
    public final GamepadButton rightBumper = new GamepadButton();
    public final GamepadButton backButton = new GamepadButton();
    public final GamepadButton startButton = new GamepadButton();
    public final GamepadButton dpadUp = new GamepadButton();
    public final GamepadButton dpadDown = new GamepadButton();
    public final GamepadButton dpadLeft = new GamepadButton();
    public final GamepadButton dpadRight = new GamepadButton();

    public final GamepadTrigger leftTrigger = new GamepadTrigger();
    public final GamepadTrigger rightTrigger = new GamepadTrigger();

    private final GamepadManager panelsGamepadManager;
    private final Gamepad hardwareGamepad;
    public Gamepad raw;

    public GamepadEx(GamepadManager panelsGamepadManager, Gamepad hardwareGamepad) {
        this.panelsGamepadManager = panelsGamepadManager;
        this.hardwareGamepad = hardwareGamepad;
    }

    public static GamepadEx newDriverGamepad(Gamepad gamepad1) {
        return new GamepadEx(PanelsGamepad.INSTANCE.getFirstManager(), gamepad1);
    }

    public static GamepadEx newOperatorGamepad(Gamepad gamepad2) {
        return new GamepadEx(PanelsGamepad.INSTANCE.getSecondManager(), gamepad2);
    }

    /**
     * Called every frame of the loop, it will call any bound events, and keep the button states up
     * to date.
     */
    public void update() {
        // because panels can't directly write to the gamepad when controlled via the dashboard.
        // It just makes sure to update the values
        raw = panelsGamepadManager.asCombinedFTCGamepad(hardwareGamepad);

        btnA.update(raw.a || raw.cross);
        btnB.update(raw.b || raw.circle);
        btnX.update(raw.x || raw.square);
        btnY.update(raw.y || raw.triangle);
        leftBumper.update(raw.left_bumper);
        rightBumper.update(raw.right_bumper);
        backButton.update(raw.back || raw.share);
        startButton.update(raw.start || raw.options);
        dpadUp.update(raw.dpad_up);
        dpadDown.update(raw.dpad_down);
        dpadLeft.update(raw.dpad_left);
        dpadRight.update(raw.dpad_right);
        leftTrigger.update(raw.left_trigger);
        rightTrigger.update(raw.right_trigger);
    }
}
