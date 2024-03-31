package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.Hardware.Intakes.ServoClaw;

@Config
public class TenacityClaw {
    public static double CLAW_LEFT_OPEN = 0.4;
    public static double CLAW_RIGHT_OPEN = 0.45;
    public static double CLAW_LEFT_CLOSED = 0.9;
    public static double CLAW_RIGHT_CLOSED = 0;

    private final ServoClaw clawLeft, clawRight;
    private final Gamepad gamepad;
    private boolean lastToggleLeftBumper = false, lastToggleRightBumper = false;

    public TenacityClaw(Gamepad gamepad, HardwareMap hardwareMap) {
        clawLeft = new ServoClaw(hardwareMap.get(Servo.class, "clawLeft"), CLAW_LEFT_OPEN, CLAW_LEFT_CLOSED);
        clawRight = new ServoClaw(hardwareMap.get(Servo.class, "clawRight"), CLAW_RIGHT_OPEN, CLAW_RIGHT_CLOSED);
        this.gamepad = gamepad;
    }

    public void updateClaw() {
        if (gamepad.left_bumper && !lastToggleLeftBumper) {
            clawLeft.toggle();
            lastToggleLeftBumper = true;
        } else if (!gamepad.left_bumper && lastToggleLeftBumper) {
            lastToggleLeftBumper = false;
        }

        if (gamepad.right_bumper && !lastToggleRightBumper) {
            clawRight.toggle();
            lastToggleRightBumper = true;
        } else if (!gamepad.right_bumper && lastToggleRightBumper) {
            lastToggleRightBumper = false;
        }
    }

    public void setClawLeftOpen(boolean isOpen) {
        clawLeft.setOpen(isOpen);
    }

    public void setClawRightOpen(boolean isOpen) {
        clawRight.setOpen(isOpen);
    }
}
