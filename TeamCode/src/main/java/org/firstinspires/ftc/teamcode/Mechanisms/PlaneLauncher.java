package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class PlaneLauncher {
    public Servo launcher;

    public enum LauncherState{
        CLOSED,
        LAUNCHED
    }
    public LauncherState launcherState;

    public static double CLOSED = 0.67;
    public static double LAUNCHED = 0.7;
    public PlaneLauncher(HardwareMap hardwareMap){
        launcher = hardwareMap.get(Servo.class, "launcher");

        launcherState = LauncherState.CLOSED;
    }

    public void setLauncher(){
        switch (launcherState){
            case CLOSED:
                launcher.setPosition(CLOSED);
                break;
            case LAUNCHED:
                launcher.setPosition(LAUNCHED);
                break;
        }
    }
}
