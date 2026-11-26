package org.firstinspires.ftc.teamcode.configs;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;

public class StandardKeyBinds {
    public StandardKeyBinds(BotHardware botHardware, Drivetrain drivetrain, GamepadWrapper driverGamepad, GamepadWrapper shooterGamepad) {
        driverGamepad.leftTrigger /// Speed Modifier
                .onPress(() -> drivetrain.speed(0.5))
                .onRelease(() -> drivetrain.speed(1));

        shooterGamepad.rightTrigger /// Feeder Toggle
                .onPress(() -> botHardware.feeder.setPower(1))
                .onRelease(() -> botHardware.feeder.setPower(0));

        shooterGamepad.leftTrigger  /// Intake Toggle
                .onPress(() -> botHardware.intake.setPower(1))
                .onRelease(() -> botHardware.intake.setPower(0));

        shooterGamepad.btnA /// Reverse
                .onPress(() -> botHardware.intake.setPower(-1))
                .onPress(() -> botHardware.intake.setPower(-1))
                .onRelease(() -> botHardware.feeder.setPower(0))
                .onRelease(() -> botHardware.feeder.setPower(0));

        shooterGamepad.leftBumper /// Servo Position
                .onPress(() -> botHardware.servo.setPosition(
                        botHardware.servo.getPosition() == Globals.SERVO_BIG_BALL ?
                                Globals.SERVO_SMALL_BALL : Globals.SERVO_BIG_BALL));
    }
}
