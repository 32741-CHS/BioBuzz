package org.firstinspires.ftc.teamcode.configs;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class DynamicBotHardware {
    /// NOT IMPLIMENTED YET!!!
    public GenericHardwareWrapper flDrive, frDrive, blDrive, brDrive, turret, flywheel, feeder, intake, imu, vision, insideCam, pinpoint, servo;


    public DynamicBotHardware(HardwareMap hwMap) {
        DcMotorSimple.Direction
                dirF = DcMotor.Direction.FORWARD,
                dirR = DcMotor.Direction.REVERSE;
        DcMotor.ZeroPowerBehavior
                brake = DcMotor.ZeroPowerBehavior.BRAKE,
                freeRoll = DcMotor.ZeroPowerBehavior.FLOAT;
        DcMotor.RunMode
                nEn = DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                yEn = DcMotor.RunMode.RUN_USING_ENCODER;


        this.flDrive = new MotorHardwareObj(hwMap, "flDrive", dirF, brake, false, new PIDFCoefficients());
        this.frDrive = new MotorHardwareObj(hwMap, "frDrive", dirF, brake, nEn);
        this.blDrive = new MotorHardwareObj(hwMap, "blDrive", dirF, brake, nEn);
        this.brDrive = new MotorHardwareObj(hwMap, "brDrive", dirF, brake, nEn);
        this.turret = new MotorHardwareObj(hwMap, "turret", dirF, brake, yEn);
        this.flywheel = new MotorHardwareObj(hwMap, "flywheel", dirF, freeRoll, yEn);
        this.feeder = new MotorHardwareObj(hwMap, "feeder", dirF, brake, nEn);
        this.intake = new MotorHardwareObj(hwMap, "intake", dirF, brake, nEn);
        this.imu = new GenericHardwareWrapper<>(hwMap, IMU.class, "imu");
        this.vision = new GenericHardwareWrapper<>(hwMap, WebcamName.class, "vision");
        this.insideCam = new GenericHardwareWrapper<>(hwMap, WebcamName.class, "insideCam");
        this.pinpoint = new GenericHardwareWrapper<>(hwMap, GoBildaPinpointDriver.class, "pinpoint");
        this.servo = new GenericHardwareWrapper<>(hwMap, Servo.class, "servo");
    }

    public static class GenericHardwareWrapper<hardwareInterfaceOrClass> {

        private final Class<? extends hardwareInterfaceOrClass> classOrInstance;
        private final String name;
        private final HardwareMap ftcHardwareMap;

        GenericHardwareWrapper(HardwareMap ftcHardwareMap, Class<? extends hardwareInterfaceOrClass> classOrInstance, String name) {
            this.ftcHardwareMap = ftcHardwareMap;
            this.classOrInstance = classOrInstance;
            this.name = name;
        }

        public hardwareInterfaceOrClass get() {
            if (ftcHardwareMap == null)
                throw new RuntimeException("Did you forget to init DynamicBotHardware?");
            return ftcHardwareMap.get(classOrInstance, name);
        }

        public String name() {
            return name;
        }
    }

    public static class MotorHardwareObj extends GenericHardwareWrapper<DcMotor> {

        private final @Nullable PIDFCoefficients constantsPIDF;
        private final DcMotorSimple.Direction direction;
        private final DcMotor.ZeroPowerBehavior zeroPowerBehavior;
        private final DcMotor.RunMode runMode;

        public MotorHardwareObj(HardwareMap hwMap, String name, DcMotorSimple.Direction direction, DcMotor.ZeroPowerBehavior zeroPowerBehavior, boolean isVelNotPosPIDF, PIDFCoefficients constantsPIDF) {
            super(hwMap, DcMotor.class, name);
            this.constantsPIDF = constantsPIDF;
            this.runMode = isVelNotPosPIDF ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_TO_POSITION;
            this.direction = direction;
            this.zeroPowerBehavior = zeroPowerBehavior;
        }

        public MotorHardwareObj(HardwareMap hwMap, String name, DcMotorSimple.Direction direction, DcMotor.ZeroPowerBehavior zeroPowerBehavior, DcMotor.RunMode runMode) {
            super(hwMap, DcMotor.class, name);
            this.runMode = runMode;
            this.constantsPIDF = null;
            this.direction = direction;
            this.zeroPowerBehavior = zeroPowerBehavior;
        }

        @Override
        public DcMotor get() {
            DcMotor motor = super.get();
            motor.setDirection(direction);
            motor.setZeroPowerBehavior(zeroPowerBehavior);
            if (constantsPIDF != null)
                ((DcMotorEx) motor).setPIDFCoefficients(runMode, constantsPIDF);
            return motor;
        }

        public PIDFCoefficients getPIDF() {
            return constantsPIDF;
        }
    }
}

