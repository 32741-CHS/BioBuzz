package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.Globals.TRIGGER_THRESHOLD;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class GamepadWrapper implements SubsystemBase {

    public final GamepadButtonWrapper
            btnA = new GamepadButtonWrapper(),
            btnB = new GamepadButtonWrapper(),
            btnX = new GamepadButtonWrapper(),
            btnY = new GamepadButtonWrapper(),
            leftBumper = new GamepadButtonWrapper(),
            rightBumper = new GamepadButtonWrapper(),
            backButton = new GamepadButtonWrapper(),
            startButton = new GamepadButtonWrapper(),
            dpadUp = new GamepadButtonWrapper(),
            dpadDown = new GamepadButtonWrapper(),
            dpadLeft = new GamepadButtonWrapper(),
            dpadRight = new GamepadButtonWrapper();

    public final GamepadTriggerWrapper
            leftTrigger = new GamepadTriggerWrapper(),
            rightTrigger = new GamepadTriggerWrapper();

    private final GamepadManager panelsGamepadManager;
    private final Gamepad hardwareGamepad;
    public Gamepad raw;

    private GamepadWrapper(GamepadManager panelsGamepadManager, Gamepad hardwareGamepad) {
        this.panelsGamepadManager = panelsGamepadManager;
        this.hardwareGamepad = hardwareGamepad;
        raw = panelsGamepadManager.asCombinedFTCGamepad(hardwareGamepad);
    }

    public static GamepadWrapper newDriverGamepad(Gamepad gamepad1) {
        return new GamepadWrapper(PanelsGamepad.INSTANCE.getFirstManager(), gamepad1);
    }

    public static GamepadWrapper newOperatorGamepad(Gamepad gamepad2) {
        return new GamepadWrapper(PanelsGamepad.INSTANCE.getSecondManager(), gamepad2);
    }

    @Override
    public void init(RobotHardwareGroup robotHardware) {
    }

    /**
     * Called every frame of the loop, it will call any bound events, and keep the button states up
     * to date.
     */
    @Override
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

    public static class GamepadButtonWrapper {
        protected final Binder callbackBinds = new Binder();
        protected boolean currentState;
        protected boolean prevState = false;

        /**
         * The callback function is called only when the button goes down, will be like a pulse. Sort of
         * like onclick in js event listeners
         *
         * @param callback The action you want to perform if the button goes down.
         */
        public GamepadButtonWrapper onPress(Runnable callback) {
            callbackBinds.bind("WAS_PRESSED", callback);
            return this;
        }

        /**
         * When the button gets released, it calls the callback function once. So call every time it is
         * released
         *
         * @param callback The function that gets called when the button is released
         */
        public GamepadButtonWrapper onRelease(Runnable callback) {
            callbackBinds.bind("WAS_RELEASED", callback);
            return this;
        }

        /**
         * A bit special where when the button is down it continuously calls the callback function EVERY
         * FRAME
         *
         * @param callback The function to run EVERY FRAME WHEN BUTTON DOWN
         */
        public GamepadButtonWrapper whenHeld(Runnable callback) {
            callbackBinds.bind("IS_HELD", callback);
            return this;
        }

        /**
         * The opposite of whenHeld, called EVERY FRAME when the button is released or relaxed position.
         *
         * @param callback The function to call
         */
        public GamepadButtonWrapper whenNotHeld(Runnable callback) {
            callbackBinds.bind("IS_NOT_HELD", callback);
            return this;
        }

        /**
         * Called every frame by the GamepadWrapper class to pass the latest button states, and call the
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

        /**
         * True the frame you just pressed it
         */
        public boolean wasPressed() {
            return currentState && !prevState;
        }

        /**
         * true the frame you just let go
         */
        public boolean wasReleased() {
            return !currentState && prevState;
        }

        /**
         * true while held
         */
        public boolean isHeld() {
            return currentState;
        }
    }

    public static class GamepadTriggerWrapper extends GamepadButtonWrapper {
        private float percentPressed = 0;

        public float getPercentPressed() {
            return percentPressed;
        }

        public void update(float currentPercentPressed) {
            percentPressed = currentPercentPressed;
            super.update(percentPressed >= TRIGGER_THRESHOLD);
        }
    }

    public static class Binder {
        private final Map<String, List<Runnable>> binds = new HashMap<>();

        /**
         * Binds an anonymous function to a key. Supports binding multiple callbacks to the same key.
         *
         * @param key      The string id, is case-sensitive and will the one you put in the call method to
         *                 later call the thing
         * @param callback The function that will be called when you do Binder.call(key)
         */
        public void bind(String key, Runnable callback) {
            // Automatically creates a new ArrayList if the key doesn't exist yet and store to the
            // hashmap, then adds the callback to the list
            binds.computeIfAbsent(key, k -> new ArrayList<>()).add(callback);
        }

        /**
         * Executes all anonymous functions registered under the given key. If the key does not exist,
         * it safely does nothing.
         *
         * @param key The key you defined in the bind method.
         */
        public void call(String key) {
            // Safely retrieves the list, defaulting to an empty list if the key is missing
            List<Runnable> callbacks = binds.getOrDefault(key, Collections.emptyList());

            // Loop through and fire each runnable sequentially
            assert callbacks != null; // This will never be false, but IDE insist.
            for (Runnable callback : callbacks) {
                callback.run();
            }
        }
    }
}
