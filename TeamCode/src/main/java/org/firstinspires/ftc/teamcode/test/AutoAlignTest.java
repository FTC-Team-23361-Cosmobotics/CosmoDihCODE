package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.teleop.systems.Drive;
import org.firstinspires.ftc.teamcode.teleop.systems.Vision;

import java.util.List;

@Config
@TeleOp
public class AutoAlignTest extends OpMode {
    public Drive drive;

    public Vision vision;

    public double distance;

    public double headingErr;



    @Override
    public void init() {
        drive = new Drive(hardwareMap, true, 0);
        vision = new Vision(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        vision.StartVision();
    }

    @Override
    public void loop() {
        drive.update(gamepad1, gamepad2, 1);
        vision.update(drive.botHeading);

        double angleToGoal = Math.toRadians(limelightMountAngleDegrees + Vision.tY);
        double distanceFromLimelightToTagInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoal);
        double b = pointOffset;
        double z = distanceFromLimelightToTagInches;
        double thetaIMU = drive.botHeading + Math.PI;
        double thetaY = Math.toRadians(Vision.tY);
        double thetaAlpha = thetaY + thetaIMU;
        double thetaSigma = Math.toRadians(180) - thetaAlpha;
        double thetaX = Math.toRadians(135) - thetaSigma;
        double thetaA = thetaX + Math.toRadians(90);
        double aRadicand = Math.pow(z, 2) + Math.pow(b, 2) - (2 * z * b * Math.cos(thetaA));
        double a = Math.sqrt(aRadicand);
        double numerator = b * Math.sin(thetaA);
        double arcSinArgument = (numerator / a);
        double thetaB = Math.asin(arcSinArgument);
        double thetaLamda = thetaB + thetaSigma;
        double thetaLamdaDegrees = Math.toDegrees(-thetaLamda);

        telemetry.addData("tx", Vision.tX);
        telemetry.addData("ty", Vision.tY);
        telemetry.addData("tA", Vision.tA);
        telemetry.addData("botHeading", Math.toDegrees(drive.botHeading));
        telemetry.addData("Numerator", numerator);
        telemetry.addData("arcSingArgument", arcSinArgument);
        telemetry.addData("ThetaB", thetaB);
        telemetry.addData("ThetaLamda", thetaLamda);
        telemetry.addData("ThetaLamda Degrees", thetaLamdaDegrees);
        telemetry.addData("Error", thetaLamdaDegrees - Math.toDegrees(drive.botHeading));

        telemetry.update();
    }

    public double rawVelocity(double ta) {
        double scale = 657.26144;
        double velocity = scale * Math.pow(ta, -2);
        return velocity;
    }

    public final double pointOffset = 8;
    public double limelightMountAngleDegrees = 0;
    public double limelightLensHeightInches = 17.1654;
    public double goalHeightInches = 29.5;
}

