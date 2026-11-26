package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;

import java.util.function.Consumer;

public class TuneWithGamepad {
    private double value;

    public TuneWithGamepad(double sensitivity, GamepadWrapper.GamepadButtonWrapper increase, GamepadWrapper.GamepadButtonWrapper reduce, Consumer<Double> updateVal) {
        increase.onPress(() -> {
            value += sensitivity;
            updateVal.accept(value);
        });
        reduce.onPress(() -> {
            value -= sensitivity;
            updateVal.accept(value);
        });
    }
}
