package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.Hardware.Arms.ArmExtensions.ArmWrist;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class TenacityWrist extends ArmWrist {
    Gamepad gamepad1;
    Telemetry telemetry;

    public enum WristPosition {
        INIT,
        INTAKE_POSITION,
        AUTON_INTAKE,
        AUTON_AFTER_INTAKE,
        OUTTAKE_FIRST_HALF,
        OUTTAKE_FIRST_LINE,
        OUTTAKE_THIRD_HALF,
        OUTTAKE_SECOND_LINE,
        OUTTAKE_FIFTH_HALF,
        OUTTAKE_THIRD_LINE,
        HANG
    }

    public static double WRIST_INTAKE = 0.53;
    public static double WRIST_AUTON_INTAKE = 0.565;
    public static double WRIST_AUTON_AFTER_INTAKE = 0.3;
    public static double WRIST_OUTTAKE_HALF1 = 0.42;
    public static double WRIST_OUTTAKE_1 = 0.45;
    public static double WRIST_OUTTAKE_HALF3 = 0.45;
    public static double WRIST_OUTTAKE_2 = 0.5;
    public static double WRIST_OUTTAKE_HALF5 = 0.5;
    public static double WRIST_OUTTAKE_3 = 0.5;
    public static double WRIST_INIT = 0.87;
    public static double WRIST_DRIVING = 0.05;
    public static double WRIST_HANG = 0.4;

    public WristPosition wristPosition = WristPosition.INIT;

    public TenacityWrist(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        super("wrist", hardwareMap);

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    public void setWristPosition() {
        switch (wristPosition) {
            case INTAKE_POSITION:
                wristServo.setPosition(WRIST_INTAKE);
                break;
            case AUTON_INTAKE:
                wristServo.setPosition(WRIST_AUTON_INTAKE);
                break;
            case AUTON_AFTER_INTAKE:
                wristServo.setPosition(WRIST_AUTON_AFTER_INTAKE);
                break;
            case INIT:
                wristServo.setPosition(WRIST_INIT);
                break;
            case OUTTAKE_FIRST_HALF:
                wristServo.setPosition(WRIST_OUTTAKE_HALF1);
                break;
            case OUTTAKE_FIRST_LINE:
                wristServo.setPosition(WRIST_OUTTAKE_1);
                break;
            case OUTTAKE_THIRD_HALF:
                wristServo.setPosition(WRIST_OUTTAKE_HALF3);
                break;
            case OUTTAKE_SECOND_LINE:
                wristServo.setPosition(WRIST_OUTTAKE_2);
                break;
            case OUTTAKE_FIFTH_HALF:
                wristServo.setPosition(WRIST_OUTTAKE_HALF5);
                break;
            case OUTTAKE_THIRD_LINE:
                wristServo.setPosition(WRIST_OUTTAKE_3);
                break;
            case HANG:
                wristServo.setPosition(WRIST_HANG);
                break;
        }
    }
}
