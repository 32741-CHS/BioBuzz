package org.firstinspires.ftc.teamcode.utils;

public class GamepadButton {
    protected final Binder callbackBinds = new Binder();
    protected boolean currentState;
    protected boolean prevState = false;

    /** True the frame you just pressed it */
    public boolean wasPressed() {
        return currentState && !prevState;
    }

    /** true while held */
    public boolean isHeld() {
        return currentState;
    }

    /** true the frame you just let go */
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
