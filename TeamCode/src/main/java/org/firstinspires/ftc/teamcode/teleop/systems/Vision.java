package org.firstinspires.ftc.teamcode.teleop.systems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.isRed;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Vision {
    public static Limelight3A limelight;

    public static LLResult result;

    public static double tX;
    public static double tY;

    public static double tA;

    public static boolean validResult;

    public Vision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (isRed) {
            limelight.pipelineSwitch(0);
        } else {
            limelight.pipelineSwitch(1);
        }
        limelight.setPollRateHz(100);
        tX = 0;
        tY = 15;
        tA = 0;
    }

    public void StartVision() {
        limelight.start();
    }

    public void update(double botHeading) {
        limelight.updateRobotOrientation(botHeading); //TODO: MAY NOT BE COMPATIBLE W PINPOINT?
        result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            validResult = true;
            tX = result.getTx();
            tY = result.getTy();
            tA = result.getTa();
        } else {
            tX=0;
            validResult = false;
        }
    }
}
