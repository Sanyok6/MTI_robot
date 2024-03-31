package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.TenacityArm;
import org.firstinspires.ftc.teamcode.Mechanisms.TenacitySlides;

@TeleOp
public class ResetArm extends LinearOpMode {
    private TenacityArm arm;
    private TenacitySlides slides;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new TenacityArm(slides, gamepad1, telemetry, hardwareMap);
        slides = new TenacitySlides(arm, gamepad1, telemetry, hardwareMap);

        arm.slides = slides;
        slides.arm = arm;

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        slides.resetEncoders();
        telemetry.addLine("Encoder reset");
        telemetry.update();
    }
}
