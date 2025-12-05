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
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.teleop.systems.Drive;
import org.firstinspires.ftc.teamcode.teleop.systems.Vision;

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
    public static double shooterp = .01, shooteri = .0000001, shooterd = .0001;

    public static double shooterf = .00075;
    //shooterf = .0006
    //shooterp = .00007
    //shooterd = .0000001
    //shooteri = 0.0000001

    public static double velocityErrorTolerance;
    public static double shooterVelocityTarget;

    public static double intakePower, transferPower;

    public double shooterpid;

    public int ticks = 0;


    @Override
    public void init() {
        vision = new Vision(hardwareMap);
        drive = new Drive(hardwareMap, true, 0);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        shooterController = new PIDFController(shooterp, shooteri, shooterd, shooterf);

        shooter = new MotorEx(hardwareMap, "shooter", Motor.GoBILDA.RPM_1150);
        shooter.setRunMode(MotorEx.RunMode.RawPower);

        intake = hardwareMap.get(DcMotorImplEx.class, "intake");
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


    @Override
    public void loop() {
        vision.update(drive.botHeading);
        double targetOffsetAngle_Vertical = Vision.tY;
        double angleToGoal = Math.toRadians(limelightMountAngleDegrees + targetOffsetAngle_Vertical);
        //calculate distance
        double distanceFromLimelightToTagInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoal);
        telemetry.addData("Angle to Goal", Math.toDegrees(angleToGoal));
        telemetry.addData("DistanceFromLimelightToTagInches", distanceFromLimelightToTagInches);


        double voltageMultiplier = 12 / voltageSensor.getVoltage();
        shooter.setPositionTolerance(velocityErrorTolerance);
        shooterController.setPIDF(shooterp, shooteri, shooterd, shooterf);
        shooterpid = shooterController.calculate(shooter.getVelocity(), shooterVelocityTarget) * voltageMultiplier;
        shooter.set(shooterpid);

        intake.setPower(intakePower);
        transfer.setPower(transferPower);



        drive.update(gamepad1, gamepad2, voltageMultiplier);
//        ticks++;
//        if (ticks > 500) {
//            ticks = 0;
//        }
        telemetry.addData("Shooter PID Value", shooterpid);
        telemetry.addData("Shooter Velocity", shooter.getVelocity());
        telemetry.addData("Shooter Velocity Target", shooterVelocityTarget);
        telemetry.addData("At Setpoint", shooterController.atSetPoint());
        telemetry.addData("Intake Power", intakePower);
        telemetry.addData("Voltage", voltageSensor.getVoltage());
        telemetry.addData("tx", Vision.tX);
        telemetry.addData("ty", Vision.tY);
        telemetry.addData("tA", Vision.tA);
        telemetry.addData("botHeading", drive.botHeading);
        double rawVelocity = rawVelocity(distanceFromLimelightToTagInches);
        double interVelocity = (int) (rawVelocity / 10);
        double velocityTar = interVelocity * 10;
        telemetry.addData("Raw Velocity", rawVelocity);
        telemetry.addData("Inter Velocity", interVelocity);
        telemetry.addData("Velocity Suggestion", velocityTar);
        telemetry.update();
    }

    public double rawVelocity(double distanceFromLimelightToTagInches) {
        //TODO: TUNE
        double scale = 265;
        double velocity = scale * Math.pow(distanceFromLimelightToTagInches, .2);
        return velocity;
    }

    public final double pointOffset = 8;

    public double alignToPoint(double distanceFromLimelightToTagInches, double botHeading, double tY) {
        //TODO: MAKE SURE ALL VALUES ARE IN RADIANS FOR CALCULATIONS
        double b = pointOffset;
        double z = distanceFromLimelightToTagInches;
        double thetaIMU = botHeading;
        double thetaY = Math.toRadians(tY);
        double thetaAlpha = thetaY + thetaIMU;
        double thetaSigma = Math.toRadians(180) - thetaAlpha;
        double thetaX = Math.toRadians(135) - thetaSigma;
        double thetaA = thetaX + Math.toRadians(90);
        double aRadicand = Math.pow(z, 2) + Math.pow(b, 2) - (2 * z * b * Math.cos(thetaA));
        double a = Math.sqrt(aRadicand);
        double denominator = b * thetaA;
        double arcSinArgument = (a / denominator);
        double thetaB = Math.asin(arcSinArgument);
        double thetaLamda = thetaB + thetaSigma;
        return thetaLamda;
    }
}

