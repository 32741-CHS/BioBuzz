package org.firstinspires.ftc.teamcode.opmodes.debug_opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.GOBILDA_5203_312RPM;
import static org.firstinspires.ftc.teamcode.configs.Globals.GOBILDA_5203_6000RPM;
import static org.firstinspires.ftc.teamcode.configs.Globals.panelsTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;


@Autonomous(name = "IntakeFeederShooterTest", group = "Tests")
public class IntakeFeederShooterTest extends LinearOpMode {
    double requestedRPS = 12;

    @Override
    public void runOpMode() {
        // New robotHardware To make force fresh init and reduce risk of improper motor
        // config.

        log("HOW TO USE", "Controller/Gamepad 1 right stick changes flywheel speed/power");
        panelsTelemetry.update();
        telemetry.update();


        while (opModeInInit()) {
            telemetry.addData("state", "init");
            telemetry.update();
        }

        waitForStart();

        DcMotorEx feeder, intake, shooter;
        (feeder = (DcMotorEx) hardwareMap.get(DcMotor.class, RobotHardwareGroup.FEEDER_MOTOR_NAME)).setPower(1);
        (intake = (DcMotorEx) hardwareMap.get(DcMotor.class, RobotHardwareGroup.INTAKE_MOTOR_NAME)).setPower(1);
        (shooter = (DcMotorEx) hardwareMap.get(DcMotor.class, RobotHardwareGroup.FLYWHEEL_MOTOR_NAME)).setPower(1);


        while (opModeIsActive()) {
            shooter.setPower(requestedRPS = Math.clamp(requestedRPS - gamepad1.left_stick_y / 10, -1, 1));
            log("Shooter Requested RMP", requestedRPS);
            log("feeder Approx. RPS (assumed 312RPM config)", feeder.getVelocity() * GOBILDA_5203_312RPM);
            log("intake Approx. RPS (assumed 312RPM config)", intake.getVelocity() * GOBILDA_5203_312RPM);
            log("shooter Approx. RPS (assumed 6000RPM config)", shooter.getVelocity() * GOBILDA_5203_6000RPM);
            panelsTelemetry.update();
            telemetry.update();
        }
    }

    private void log(String key, Object value) {
        panelsTelemetry.addData(key, value);
        telemetry.addData(key, value);
    }

}
