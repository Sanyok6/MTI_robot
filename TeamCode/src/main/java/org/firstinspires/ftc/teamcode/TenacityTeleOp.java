package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.TenacityArm;
import org.firstinspires.ftc.teamcode.Mechanisms.TenacityChassis;
import org.firstinspires.ftc.teamcode.Mechanisms.TenacityClaw;
import org.firstinspires.ftc.teamcode.Mechanisms.TenacitySlides;
import org.firstinspires.ftc.teamcode.Mechanisms.TenacityWrist;

@TeleOp
public class TenacityTeleOp extends LinearOpMode {
    // TODO: add a slides manual control state
    // TODO: add a Close scoring state
    private TenacitySlides slides;
    private TenacityArm arm;
    private TenacityClaw doubleClaw;
    private TenacityWrist wrist;
    private TenacityChassis chassis;

    private ElapsedTime runtime = new ElapsedTime();

    private enum TeleOpState {
        INTAKE,
        DRIVING_ONE,
        DRIVING_TWO,
        FIRST_LINE,
        SECOND_LINE,
        THIRD_LINE,
        HANG,
        MANUAL
    }

    private enum IntakeStates {
        CLOSE,
        FAR
    }

    private IntakeStates intakeStates;
    private TeleOpState teleOpState;
    private TeleOpState scoringState;

    private boolean lastToggleX = false;
    private boolean lastToggleUp = false;
    private boolean lastToggleDown = false;
    private boolean lastToggleB = false;
    private boolean lastToggleA = false;

    @Override
    public void runOpMode() throws InterruptedException {
        slides = new TenacitySlides(arm, gamepad1, telemetry, hardwareMap);
        arm = new TenacityArm(slides, gamepad1, telemetry, hardwareMap);
        doubleClaw = new TenacityClaw(gamepad1, hardwareMap);
        wrist = new TenacityWrist(gamepad1, telemetry, hardwareMap);
        chassis = new TenacityChassis(gamepad1, hardwareMap);

        slides.arm = arm;
        arm.slides = slides;

        teleOpState = TeleOpState.INTAKE;
        scoringState = TeleOpState.FIRST_LINE;

        intakeStates = IntakeStates.FAR;

        runtime.reset();

        while (opModeInInit()) {
            slides.setSlidePower();
            wrist.setWristPosition();

            if (runtime.seconds() > 3)
                arm.setArmPower();

            if (gamepad1.start)
                slides.resetEncoders();

            telemetry.addLine("Waiting For Start");
            telemetry.update();
        }

        while (opModeIsActive()) {
            chassis.ManualDrive();

            arm.setArmPower();
            slides.setSlidePower();

            doubleClaw.updateClaw();

            wrist.setWristPosition();

            switch (teleOpState) {
                case INTAKE:
                    arm.armState = TenacityArm.ArmState.CLOSE_INTAKE;
                    wrist.wristPosition = TenacityWrist.WristPosition.INTAKE_POSITION;

                    switch (intakeStates) {
                        case FAR:
                            slides.slidesState = TenacitySlides.SlidesState.FAR_INTAKE;
                            if ((gamepad1.a != lastToggleA) && gamepad1.a) {
                                intakeStates = IntakeStates.CLOSE;
                            }

                            break;
                        case CLOSE:
                            slides.slidesState = TenacitySlides.SlidesState.INIT;
                            if ((gamepad1.a != lastToggleA) && gamepad1.a) {
                                intakeStates = IntakeStates.FAR;
                            }
                            break;
                    }

                    if ((gamepad1.x != lastToggleX) && gamepad1.x) {
                        teleOpState = scoringState;
                    }

//                    if ((gamepad1.b != lastToggleB) && gamepad1.b){
//                        teleOpState = TeleOpState.MANUAL;
//                    }

                    if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                        teleOpState = TeleOpState.DRIVING_ONE;
                    }
                    break;
                case DRIVING_ONE:
                    arm.armState = TenacityArm.ArmState.DRIVING;
                    slides.slidesState = TenacitySlides.SlidesState.INIT;
                    wrist.wristPosition = TenacityWrist.WristPosition.INIT;

                    if ((gamepad1.x != lastToggleX) && gamepad1.x) {
                        teleOpState = scoringState;
                    }

                    if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                        teleOpState = TeleOpState.INTAKE;
                    }

//                    if ((gamepad1.b != lastToggleB) && gamepad1.b){
//                        teleOpState = TeleOpState.MANUAL;
//                    }
                    break;
                case DRIVING_TWO:
                    arm.armState = TenacityArm.ArmState.DRIVING;
                    slides.slidesState = TenacitySlides.SlidesState.INIT;
                    wrist.wristPosition = TenacityWrist.WristPosition.INIT;

                    if ((gamepad1.x != lastToggleX) && gamepad1.x) {
                        teleOpState = TeleOpState.INTAKE;
                    }

                    if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                        teleOpState = scoringState;
                    }

//                    if ((gamepad1.b != lastToggleB) && gamepad1.b){
//                        teleOpState = TeleOpState.MANUAL;
//                    }
                    break;
                case FIRST_LINE:
                    arm.armState = TenacityArm.ArmState.FIRST_LINE;
                    slides.slidesState = TenacitySlides.SlidesState.FIRST_LINE;
                    wrist.wristPosition = TenacityWrist.WristPosition.OUTTAKE_FIRST_LINE;

                    if ((gamepad1.x != lastToggleX) && gamepad1.x) {
                        scoringState = teleOpState;
                        teleOpState = TeleOpState.INTAKE;
                    }

                    if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up) {
                        teleOpState = TeleOpState.SECOND_LINE;
                    }

//                    if ((gamepad1.b != lastToggleB) && gamepad1.b){
//                        teleOpState = TeleOpState.MANUAL;
//                    }

                    if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                        teleOpState = TeleOpState.DRIVING_TWO;
                    }
                    break;
                case SECOND_LINE:
                    arm.armState = TenacityArm.ArmState.SECOND_LINE;
                    slides.slidesState = TenacitySlides.SlidesState.SECOND_LINE;
                    wrist.wristPosition = TenacityWrist.WristPosition.OUTTAKE_SECOND_LINE;

                    if ((gamepad1.x != lastToggleX) && gamepad1.x) {
                        scoringState = teleOpState;
                        teleOpState = TeleOpState.INTAKE;
                    }

                    if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up) {
                        teleOpState = TeleOpState.THIRD_LINE;
                    }

                    if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down) {
                        teleOpState = TeleOpState.FIRST_LINE;
                    }

//                    if ((gamepad1.b != lastToggleB) && gamepad1.b){
//                        teleOpState = TeleOpState.MANUAL;
//                    }

                    if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                        teleOpState = TeleOpState.DRIVING_TWO;
                    }
                    break;
                case THIRD_LINE:
                    arm.armState = TenacityArm.ArmState.THIRD_LINE;
                    slides.slidesState = TenacitySlides.SlidesState.THIRD_LINE;
                    wrist.wristPosition = TenacityWrist.WristPosition.OUTTAKE_THIRD_LINE;

                    if ((gamepad1.x != lastToggleX) && gamepad1.x) {
                        scoringState = teleOpState;
                        teleOpState = TeleOpState.INTAKE;
                    }

                    if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down) {
                        teleOpState = TeleOpState.SECOND_LINE;
                    }

//                    if ((gamepad1.b != lastToggleB) && gamepad1.b){
//                        teleOpState = TeleOpState.MANUAL;
//                    }

                    if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                        teleOpState = TeleOpState.DRIVING_TWO;
                    }
                    break;
                case HANG:
                    // TODO: Figure out how the hang is gonna work
                    break;
                case MANUAL:
                    slides.slidesState = TenacitySlides.SlidesState.MANUAL;

                    if ((gamepad1.x != lastToggleX) && gamepad1.x) {
                        scoringState = teleOpState;
                        teleOpState = TeleOpState.INTAKE;
                    }
                    break;
            }

            // TODO: Plane launcher

            lastToggleX = gamepad1.x;
            lastToggleUp = gamepad1.dpad_up;
            lastToggleDown = gamepad1.dpad_down;
            lastToggleB = gamepad1.b;
            lastToggleA = gamepad1.a;

            arm.TuningTelemetry();

            telemetry.addData("Wrist Position", wrist.wristPosition);
            telemetry.update();
        }
    }
}
