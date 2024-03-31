package org.firstinspires.ftc.robotcontroller.Hardware.Intakes;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoClaw {
    private final Servo clawServo;
    private final Double openPosition, closedPosition;
    private boolean isOpen = false;

    public ServoClaw(Servo servo, Double openPosition, Double closedPosition) {
        this.clawServo = servo;
        this.openPosition = openPosition;
        this.closedPosition = closedPosition;
    }

    public void toggle() {
        isOpen = !isOpen;
        clawServo.setPosition(isOpen ? openPosition : closedPosition);
    }

    public void setOpen(boolean isOpen) {
        this.isOpen = isOpen;
        clawServo.setPosition(isOpen ? openPosition : closedPosition);
    }
}
