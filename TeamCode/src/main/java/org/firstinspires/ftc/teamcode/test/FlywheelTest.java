package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teleopV1.systems.Drive;
import org.firstinspires.ftc.teamcode.teleopV1.systems.Vision;

import java.util.List;

@Config
@TeleOp
public class FlywheelTest extends OpMode {
    public Drive drive;

    public Vision vision;
    public VoltageSensor voltageSensor;
    public MotorEx shooter;

    public DcMotorEx intake;


    public DcMotorEx transfer;
    public List<LynxModule> hubs;

    public PIDFController shooterController;
    public static double shooterp = .01, shooteri = 0, shooterd = 0;

    public static double kV = .000400186335403727, kS = .001;

    public static double shooterVelocityTarget;

    public static double intakePower, transferPower;

    public double shooterpid;

    public static double scale = 460;
    public static double slope = 1.53;

    @Override
    public void init() {
        vision = new Vision(hardwareMap);
        drive = new Drive(hardwareMap, true, 0);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        shooterController = new PIDFController(shooterp, shooteri, shooterd, kV);

        shooter = new MotorEx(hardwareMap, "shooter", Motor.GoBILDA.BARE);
        shooter.setRunMode(MotorEx.RunMode.RawPower);

        intake = hardwareMap.get(DcMotorImplEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        vision.StartVision();
    }

    public double limelightMountAngleDegrees = 0;
    // distance from the center of the Limelight lens to the floor
    public double limelightLensHeightInches = 17.1654;
    // distance from the target to the floor
    public double goalHeightInches = 29.5;

    public double voltageMultiplier = 1;
    public static double nominalVoltage = 14;


    @Override
    public void loop() {
        vision.update();
        double targetOffsetAngle_Vertical = Vision.tY;
        double angleToGoal = Math.toRadians(limelightMountAngleDegrees + targetOffsetAngle_Vertical);
        //calculate distance
        double distanceFromLimelightToTagInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoal);
        telemetry.addData("Angle to Goal", Math.toDegrees(angleToGoal));
        telemetry.addData("DistanceFromLimelightToTagInches", distanceFromLimelightToTagInches);

        double voltage = voltageSensor.getVoltage();
        voltageMultiplier = nominalVoltage / voltage;
        shooterController.setPIDF(shooterp, shooteri, shooterd, kV * voltageMultiplier);
        shooterpid = shooterController.calculate(shooter.getVelocity(), shooterVelocityTarget) + (Math.signum(shooterVelocityTarget) * kS * voltageMultiplier);
        shooter.set(Range.clip(shooterpid, -1, 1));

        intake.setPower(intakePower);
        transfer.setPower(transferPower);

        drive.update(gamepad1, gamepad2, voltage);
        telemetry.addData("Shooter PID Value", shooterpid);
        telemetry.addData("Shooter Velocity", shooter.getVelocity());
        telemetry.addData("Shooter Velocity Target", shooterVelocityTarget);
        telemetry.addData("At Setpoint", shooterController.atSetPoint());
        telemetry.addData("Intake Power", intakePower);
        telemetry.addData("Voltage", voltageSensor.getVoltage());
        telemetry.addData("voltageMultiplier", voltageMultiplier);
        telemetry.addData("tx", Vision.tX);
        telemetry.addData("ty", Vision.tY);
        telemetry.addData("botHeading", drive.botHeading);
        double rawVelocity = rawVelocity(distanceFromLimelightToTagInches);
        double interVelocity = (int) (rawVelocity / 10);
        double velocityTarget = interVelocity * 10;
        telemetry.addData("Velocity Suggestion", velocityTarget);
        telemetry.update();
    }

    public double rawVelocity(double distanceFromLimelightToTagInches) {
        return scale * Math.pow(distanceFromLimelightToTagInches, slope);
    }

//    public final double pointOffset = 8;

//    public double alignToPoint(double distanceFromLimelightToTagInches, double botHeading, double tY) {
//        //TODO: MAKE SURE ALL VALUES ARE IN RADIANS FOR CALCULATIONS
//        double b = pointOffset;
//        double z = distanceFromLimelightToTagInches;
//        double thetaIMU = botHeading;
//        double thetaY = Math.toRadians(tY);
//        double thetaAlpha = thetaY + thetaIMU;
//        double thetaSigma = Math.toRadians(180) - thetaAlpha;
//        double thetaX = Math.toRadians(135) - thetaSigma;
//        double thetaA = thetaX + Math.toRadians(90);
//        double aRadicand = Math.pow(z, 2) + Math.pow(b, 2) - (2 * z * b * Math.cos(thetaA));
//        double a = Math.sqrt(aRadicand);
//        double denominator = b * thetaA;
//        double arcSinArgument = (a / denominator);
//        double thetaB = Math.asin(arcSinArgument);
//        double thetaLamda = thetaB + thetaSigma;
//        return thetaLamda;
//    }
}

