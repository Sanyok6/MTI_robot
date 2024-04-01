package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.TenacityArm;
import org.firstinspires.ftc.teamcode.Mechanisms.TenacityChassis;
import org.firstinspires.ftc.teamcode.Mechanisms.TenacitySlides;

@TeleOp
public class ResetArm extends LinearOpMode {
    private TenacityArm arm;
    private TenacitySlides slides;
    private TenacityChassis chassis;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new TenacityArm(slides, gamepad1, telemetry, hardwareMap);
        slides = new TenacitySlides(arm, gamepad1, telemetry, hardwareMap);
        chassis = new TenacityChassis(gamepad1, hardwareMap);

        arm.slides = slides;
        slides.arm = arm;

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        slides.resetEncoders();
        chassis.resetHeading();
        telemetry.addLine("Encoder and heading reset");
        telemetry.update();
    }
}
