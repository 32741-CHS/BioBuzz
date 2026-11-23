package org.firstinspires.ftc.teamcode.opmodes.debug_opmodes;

import static org.firstinspires.ftc.teamcode.configs.Globals.GOBILDA_5203_312RPM;
import static org.firstinspires.ftc.teamcode.configs.Globals.GOBILDA_5203_6000RPM;
import static org.firstinspires.ftc.teamcode.configs.Globals.simpleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.configs.BotHardware;


@Autonomous(name = "IntakeFeederShooterTest", group = "Tests")
public class IntakeFeederShooterTest extends LinearOpMode {
    double requestedRPS = 12;
    private BotHardware botHardware;

    @Override
    public void runOpMode() {
        // New botHardware To make force fresh init and reduce risk of improper motor
        // config.
        simpleTelemetry.registerFtcTelemetry(telemetry);
        botHardware = new BotHardware(hardwareMap);

        simpleTelemetry.addData("HOW TO USE", "Controller/Gamepad 1 right stick changes flywheel speed/power");
        simpleTelemetry.update();

        while (opModeInInit()) {
            simpleTelemetry.addData("state", "init");
            simpleTelemetry.update();
        }

        waitForStart();

        DcMotorEx feeder, intake, shooter;
        (feeder = (DcMotorEx) botHardware.feeder).setPower(1);
        (intake = (DcMotorEx) botHardware.intake).setPower(1);
        (shooter = (DcMotorEx) botHardware.flywheel).setPower(1);


        while (opModeIsActive()) {
            shooter.setPower(requestedRPS = Math.clamp(requestedRPS - gamepad1.left_stick_y / 10, -1, 1));
            simpleTelemetry.addData("Shooter Requested RMP", requestedRPS);
            simpleTelemetry.addData("feeder Approx. RPS (assumed 312RPM config)", feeder.getVelocity() * GOBILDA_5203_312RPM);
            simpleTelemetry.addData("intake Approx. RPS (assumed 312RPM config)", intake.getVelocity() * GOBILDA_5203_312RPM);
            simpleTelemetry.addData("shooter Approx. RPS (assumed 6000RPM config)", shooter.getVelocity() * GOBILDA_5203_6000RPM);
            simpleTelemetry.update();
        }
    }

}
