package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcontroller.Control.PID_Controller;
import org.firstinspires.ftc.robotcontroller.Hardware.Motor;
import org.firstinspires.ftc.robotcontroller.Math.Vectors.Vector3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.util.List;

@Config
public class TenacityChassis {
    Gamepad gamepad1;

    Vector3D controllerInput = new Vector3D(0, 0, 0);


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

    private final IMU imu;
    private double targetAngle = 0;
    private boolean usingPIDControl = false;
    private static double headingOffset = 0;

    public static double TURN_SPEED = 0.5;
    public static double HEADING_TOLERANCE = 0.05;
    public static double MAX_HEADING_PID_OUTPUT = 1.5;
    public static PIDCoefficients CHASSIS_PID_COEFFS = new PIDCoefficients(0.5, 0, 0);
    private final PID_Controller chassisPID = new PID_Controller(CHASSIS_PID_COEFFS, 0);

    public TenacityChassis(Gamepad gamepad1, HardwareMap hardwareMap) {
        frontLeft = new Motor("fLeft", hardwareMap);
        frontRight = new Motor("fRight", hardwareMap);
        backRight = new Motor("bRight", hardwareMap);
        backLeft = new Motor("bLeft", hardwareMap);

        frontRight.setDirectionForward();
        backRight.setDirectionForward();
        frontLeft.setDirectionReverse();
        backLeft.setDirectionReverse();

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN)));

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

    public void resetHeading() {
        headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private double getHeading() {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingOffset;
        if (heading > Math.PI)
            heading -= 2 * Math.PI;
        else if (heading < -Math.PI)
            heading += 2 * Math.PI;
        return heading;
    }

    private double clamp(double x, double low, double high) {
        if (x < low)
            return low;
        if (x > high)
            return high;
        return x;
    }

    public void ManualDrive() {
        double forward = gamepad1.left_stick_y;
        double left = gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x;

        double heading = getHeading();

        if (rotation == 0) {
            double error = heading - targetAngle;
            if (error > Math.PI)
                error -= 2 * Math.PI;
            else if (error < -Math.PI)
                error += 2 * Math.PI;

            if (!usingPIDControl) {
                targetAngle = heading;
                usingPIDControl = true;
            }

            if (Math.abs(error) > HEADING_TOLERANCE) {
                double output = chassisPID.PID_Power(0, error);
                rotation = clamp(output, -MAX_HEADING_PID_OUTPUT, MAX_HEADING_PID_OUTPUT);
            }
        } else {
            usingPIDControl = false;
        }

        fLeft = forward - left + TURN_SPEED * rotation;
        fRight = forward + left - TURN_SPEED * rotation;
        bRight = forward - left - TURN_SPEED * rotation;
        bLeft = forward + left + TURN_SPEED * rotation;

        max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
        if (max > 1.0) {
            fLeft /= max;
            fRight /= max;
            bLeft /= max;
            bRight /= max;
        }

        setPower(fLeft, fRight, bRight, bLeft);
    }

    public void setPower(double fLeft, double fRight, double bRight, double bLeft) {
        frontLeft.setPower(fLeft);
        frontRight.setPower(fRight);
        backRight.setPower(bRight);
        backLeft.setPower(bLeft);
    }
}
