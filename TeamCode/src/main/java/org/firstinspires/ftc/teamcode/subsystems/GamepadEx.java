package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.Globals.TRIGGER_THRESHOLD;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.Binder;

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
        raw = panelsGamepadManager.asCombinedFTCGamepad(hardwareGamepad);
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

    public static class GamepadButton {
        protected final Binder callbackBinds = new Binder();
        protected boolean currentState;
        protected boolean prevState = false;

        /**
         * True the frame you just pressed it
         */
        public boolean wasPressed() {
            return currentState && !prevState;
        }

        /**
         * true while held
         */
        public boolean isHeld() {
            return currentState;
        }

        /**
         * true the frame you just let go
         */
        public boolean wasReleased() {
            return !currentState && prevState;
        }

        /**
         * The callback function is called only when the button goes down, will be like a pulse. Sort of
         * like onclick in js event listeners
         *
         * @param callback The action you want to perform if the button goes down.
         */
        public void onPress(Runnable callback) {
            callbackBinds.bind("WAS_PRESSED", callback);
        }

        /**
         * When the button gets released, it calls the callback function once. So call every time it is
         * released
         *
         * @param callback The function that gets called when the button is released
         */
        public void onRelease(Runnable callback) {
            callbackBinds.bind("WAS_RELEASED", callback);
        }

        /**
         * A bit special where when the button is down it continuously calls the callback function EVERY
         * FRAME
         *
         * @param callback The function to run EVERY FRAME WHEN BUTTON DOWN
         */
        public void whenHeld(Runnable callback) {
            callbackBinds.bind("IS_HELD", callback);
        }

        /**
         * The opposite of whenHeld, called EVERY FRAME when the button is released or relaxed position.
         *
         * @param callback The function to call
         */
        public void whenNotHeld(Runnable callback) {
            callbackBinds.bind("IS_NOT_HELD", callback);
        }

        /**
         * Called every frame by the GamepadEx class to pass the latest button states, and call the
         * callbacks
         *
         * @param pressed Whether the button is pressed or not at this moment of time
         */
        public void update(boolean pressed) {
            prevState = currentState;
            currentState = pressed;

            if (wasPressed()) callbackBinds.call("WAS_PRESSED");
            if (wasReleased()) callbackBinds.call("WAS_RELEASED");
            if (isHeld()) callbackBinds.call("IS_HELD");
            if (!isHeld()) callbackBinds.call("IS_NOT_HELD");
        }
    }

    public static class GamepadTrigger extends GamepadButton {
        private float percentPressed = 0;

        public float getPercentPressed() {
            return percentPressed;
        }

        public void update(float currentPercentPressed) {
            percentPressed = currentPercentPressed;
            super.update(percentPressed >= TRIGGER_THRESHOLD);
        }
    }
}
