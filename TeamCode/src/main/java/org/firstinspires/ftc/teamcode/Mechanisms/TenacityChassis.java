package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcontroller.Hardware.Motor;
import org.firstinspires.ftc.robotcontroller.Math.Vectors.Vector2D;
import org.firstinspires.ftc.robotcontroller.Math.Vectors.Vector3D;

import java.util.Arrays;
import java.util.List;

public class TenacityChassis {
    Gamepad gamepad1;

    Vector3D controllerInput = new Vector3D(0,0,0);


    public double fLeft;
    public double fRight;
    public double bLeft;
    public double bRight;

    public Motor frontLeft;
    public Motor frontRight;
    public Motor backRight;
    public Motor backLeft;

    private List<Motor> motors;
    public double max;

    public TenacityChassis(Gamepad gamepad1, HardwareMap hardwareMap) {
        frontLeft = new Motor("fLeft", hardwareMap);
        frontRight = new Motor("fRight", hardwareMap);
        backRight = new Motor("bRight", hardwareMap);
        backLeft = new Motor("bLeft", hardwareMap);

        frontRight.setDirectionForward();
        backRight.setDirectionForward();
        frontLeft.setDirectionReverse();
        backLeft.setDirectionReverse();

        motors = Arrays.asList(frontLeft, frontRight, backRight, backLeft);

        for (Motor motor : motors) {
            motor.setBrakeMode();
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        this.gamepad1 = gamepad1;
    }

    public void ManualDrive(){
        controllerInput.set(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        setDriveVectorsRobotCentric(controllerInput);
    }

    public void SkidSteerDrive(){
        skidSteer(new Vector2D(gamepad1.left_stick_y, gamepad1.right_stick_y));
    }
    public void skidSteer(Vector2D input){
        fLeft = input.B;
        bLeft = input.B;

        fRight = input.A;
        bRight = input.A;

        max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
        if (max > 1.0) {
            fLeft /= max;
            fRight /= max;
            bLeft /= max;
            bRight /= max;
        }

        setPower(fLeft, fRight, bRight, bLeft);
    }

    public void setDriveVectorsRobotCentric(Vector3D input){
        fLeft = input.A -   input.B +   input.C;
        fRight = input.A +   input.B -   input.C;
        bRight = input.A -   input.B -   input.C;
        bLeft = input.A +   input.B +   input.C;

        max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
        if (max > 1.0) {
            fLeft /= max;
            fRight /= max;
            bLeft /= max;
            bRight /= max;
        }

        setPower(fLeft, fRight, bRight, bLeft);
    }


    public void setPower(double power){
        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
        this.backRight.setPower(power);
        this.backLeft.setPower(power);
    }

    public void setPower(double fLeft, double fRight, double bRight, double bLeft){
        this.frontLeft.setPower(fLeft);
        this.frontRight.setPower(fRight);
        this.backRight.setPower(bRight);
        this.backLeft.setPower(bLeft);
    }
}
