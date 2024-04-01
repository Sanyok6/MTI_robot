package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.PlaneLauncher;
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
    private PlaneLauncher launcher;

    private ElapsedTime runtime = new ElapsedTime();

    private enum TeleOpState {
        INTAKE,
        DRIVING,
        FIRST_HALF,
        FIRST_LINE,
        THIRD_HALF,
        SECOND_LINE,
        FIFTH_HALF,
        THIRD_LINE,
        HANG,
        MANUAL
    }

    private enum IntakeStates {
        CLOSE,
        FAR
    }

    enum HangStates{
        ALIGN,
        PULL
    }

    private HangStates hangState;
    private IntakeStates intakeStates;
    private TeleOpState teleOpState;
    private TeleOpState scoringState;

    private boolean lastToggleX = false;
    private boolean lastToggleUp = false;
    private boolean lastToggleDown = false;
    private boolean lastToggleB = false;
    private boolean lastToggleA = false;
    private boolean lastToggleOption = false;

    @Override
    public void runOpMode() throws InterruptedException {
        slides = new TenacitySlides(arm, gamepad1, telemetry, hardwareMap);
        arm = new TenacityArm(slides, gamepad1, telemetry, hardwareMap);
        doubleClaw = new TenacityClaw(gamepad1, hardwareMap);
        wrist = new TenacityWrist(gamepad1, telemetry, hardwareMap);
        chassis = new TenacityChassis(gamepad1, hardwareMap);
        launcher = new PlaneLauncher(hardwareMap);

        slides.arm = arm;
        arm.slides = slides;

        teleOpState = TeleOpState.DRIVING;
        scoringState = TeleOpState.FIRST_HALF;

        hangState = HangStates.ALIGN;

        intakeStates = IntakeStates.FAR;

        runtime.reset();

        while (opModeInInit()) {
            slides.setSlidePower();
            wrist.setWristPosition();

            if (runtime.seconds() > 3)
                arm.setArmPower();

            telemetry.addLine("Waiting For Start");
            telemetry.update();
        }

        while (opModeIsActive()) {
            chassis.ManualDrive();

            arm.setArmPower();
            slides.setSlidePower();

            launcher.setLauncher();

            doubleClaw.updateClaw();

            wrist.setWristPosition();

            if (gamepad1.back){
                launcher.launcherState = PlaneLauncher.LauncherState.LAUNCHED;
            }

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

                    if ((gamepad1.options != lastToggleOption) && gamepad1.options){
                        teleOpState = TeleOpState.HANG;
                    }
                    if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                        teleOpState = TeleOpState.DRIVING;
                    }
                    break;
                case DRIVING:
                    arm.armState = TenacityArm.ArmState.DRIVING;
                    slides.slidesState = TenacitySlides.SlidesState.INIT;
                    wrist.wristPosition = TenacityWrist.WristPosition.INIT;

                    if ((gamepad1.x != lastToggleX) && gamepad1.x) {
                        teleOpState = scoringState;
                    }

                    if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                        teleOpState = TeleOpState.INTAKE;
                    }

                    if ((gamepad1.options != lastToggleOption) && gamepad1.options){
                        teleOpState = TeleOpState.HANG;
                    }
                    break;
                case FIRST_HALF:
                    arm.armState = TenacityArm.ArmState.FIRST_HALF;
                    slides.slidesState = TenacitySlides.SlidesState.FIRST_HALF;
                    wrist.wristPosition = TenacityWrist.WristPosition.OUTTAKE_FIRST_HALF;

                    if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up) {
                        teleOpState = TeleOpState.FIRST_LINE;
                    }

                    if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                        teleOpState = TeleOpState.DRIVING;
                        scoringState = TeleOpState.FIRST_HALF;
                    }

                    if ((gamepad1.options != lastToggleOption) && gamepad1.options){
                        teleOpState = TeleOpState.HANG;
                    }
                    break;
                case FIRST_LINE:
                    arm.armState = TenacityArm.ArmState.FIRST_LINE;
                    slides.slidesState = TenacitySlides.SlidesState.FIRST_LINE;
                    wrist.wristPosition = TenacityWrist.WristPosition.OUTTAKE_FIRST_LINE;

                    if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up) {
                        teleOpState = TeleOpState.THIRD_HALF;
                    }

                    if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                        teleOpState = TeleOpState.DRIVING;
                        scoringState = TeleOpState.FIRST_LINE;
                    }

                    if ((gamepad1.options != lastToggleOption) && gamepad1.options){
                        teleOpState = TeleOpState.HANG;
                    }

                    if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down) {
                        teleOpState = TeleOpState.FIRST_HALF;
                    }
                    break;
                case THIRD_HALF:
                    arm.armState = TenacityArm.ArmState.THIRD_HALF;
                    slides.slidesState = TenacitySlides.SlidesState.THIRD_HALF;
                    wrist.wristPosition = TenacityWrist.WristPosition.OUTTAKE_THIRD_HALF;

                    if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up) {
                        teleOpState = TeleOpState.SECOND_LINE;
                    }

                    if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                        teleOpState = TeleOpState.DRIVING;
                        scoringState = TeleOpState.THIRD_HALF;
                    }

                    if ((gamepad1.options != lastToggleOption) && gamepad1.options){
                        teleOpState = TeleOpState.HANG;
                    }

                    if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down) {
                        teleOpState = TeleOpState.FIRST_LINE;
                    }
                    break;
                case SECOND_LINE:
                    arm.armState = TenacityArm.ArmState.SECOND_LINE;
                    slides.slidesState = TenacitySlides.SlidesState.SECOND_LINE;
                    wrist.wristPosition = TenacityWrist.WristPosition.OUTTAKE_SECOND_LINE;

                    if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up) {
                        teleOpState = TeleOpState.FIFTH_HALF;
                    }

                    if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down) {
                        teleOpState = TeleOpState.THIRD_HALF;
                    }

                    if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                        teleOpState = TeleOpState.DRIVING;
                        scoringState = TeleOpState.SECOND_LINE;
                    }

                    if ((gamepad1.options != lastToggleOption) && gamepad1.options){
                        teleOpState = TeleOpState.HANG;
                    }
                    break;
                case FIFTH_HALF:
                    arm.armState = TenacityArm.ArmState.FIFTH_HALF;
                    slides.slidesState = TenacitySlides.SlidesState.FIFTH_HALF;
                    wrist.wristPosition = TenacityWrist.WristPosition.OUTTAKE_FIFTH_HALF;

                    if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up) {
                        teleOpState = TeleOpState.THIRD_LINE;
                    }

                    if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                        teleOpState = TeleOpState.DRIVING;
                        scoringState = TeleOpState.FIFTH_HALF;
                    }

                    if ((gamepad1.options != lastToggleOption) && gamepad1.options){
                        teleOpState = TeleOpState.HANG;
                    }

                    if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down) {
                        teleOpState = TeleOpState.SECOND_LINE;
                    }
                    break;
                case THIRD_LINE:
                    arm.armState = TenacityArm.ArmState.THIRD_LINE;
                    slides.slidesState = TenacitySlides.SlidesState.THIRD_LINE;
                    wrist.wristPosition = TenacityWrist.WristPosition.OUTTAKE_THIRD_LINE;

                    if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down) {
                        teleOpState = TeleOpState.FIFTH_HALF;
                    }

                    if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                        teleOpState = TeleOpState.DRIVING;
                        scoringState = TeleOpState.THIRD_LINE;
                    }

                    if ((gamepad1.options != lastToggleOption) && gamepad1.options){
                        teleOpState = TeleOpState.HANG;
                    }
                    break;
                case HANG:
                    switch (hangState){
                        case ALIGN:
                            arm.armState = TenacityArm.ArmState.HANG;
                            slides.slidesState = TenacitySlides.SlidesState.HANG_ALIGN;
                            wrist.wristPosition = TenacityWrist.WristPosition.HANG;

                            if ((gamepad1.x != lastToggleX) && gamepad1.x){
                                hangState = HangStates.PULL;
                            }
                            break;
                        case PULL:
                            arm.armState = TenacityArm.ArmState.HANG;
                            slides.slidesState = TenacitySlides.SlidesState.HANG_PULL;
                            wrist.wristPosition = TenacityWrist.WristPosition.HANG;

                            if ((gamepad1.x != lastToggleX) && gamepad1.x){
                                hangState = HangStates.ALIGN;
                            }

                            if ((gamepad1.options != lastToggleOption) && gamepad1.options){
                                teleOpState = TeleOpState.INTAKE;
                            }
                            break;
                    }
                    break;
                case MANUAL:
                    slides.slidesState = TenacitySlides.SlidesState.MANUAL;

                    if ((gamepad1.x != lastToggleX) && gamepad1.x) {
                        teleOpState = TeleOpState.INTAKE;
                    }

                    if ((gamepad1.options != lastToggleOption) && gamepad1.options){
                        teleOpState = TeleOpState.HANG;
                    }
                    break;
            }

            // TODO: Plane launcher

            lastToggleX = gamepad1.x;
            lastToggleUp = gamepad1.dpad_up;
            lastToggleDown = gamepad1.dpad_down;
            lastToggleB = gamepad1.b;
            lastToggleA = gamepad1.a;
            lastToggleOption = gamepad1.options;

            telemetry.addData("Arm State", arm.armState);
            arm.TuningTelemetry();

            telemetry.update();
        }
    }
}
