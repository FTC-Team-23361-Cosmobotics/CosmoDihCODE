package org.firstinspires.ftc.teamcode.teleopV1.systems;

import static org.firstinspires.ftc.teamcode.teleopV1.utils.GlobalVars.isRed;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Vision {
    public static Limelight3A limelight;

    public static LLResult result;

    public static double tX;
    public static double tY;

    public static boolean validResult;

    public static boolean isConnected = false;

    public static boolean isRunning = false;

    public Vision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        isConnected = limelight.isConnected();
        isRunning = limelight.isRunning();
        if (isRed) {
            limelight.pipelineSwitch(0);
        } else {
            limelight.pipelineSwitch(1);
        }
        limelight.setPollRateHz(100);
        tX = 0;
        tY = 15;
    }

    public void StartVision() {
        limelight.start();
    }

    public void update() {
        isConnected = limelight.isConnected();
        isRunning = limelight.isRunning();
        result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            validResult = true;
            tX = result.getTx();
            tY = result.getTy();
        } else {
            tX = 0;
            validResult = false;
        }
    }
}
