//TODO: POSSIBLY IMPLEMENT PEDRO PATHING DRIVE FOR MORE ACCURATE FIELDCENTRIC IMU/SMOOTHER DRIVE
package org.firstinspires.ftc.teamcode.teleop.systems;



import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.isRed;
import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.transitionHeading;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;
import org.firstinspires.ftc.teamcode.test.pinpoint.GoBildaPinpointDriver;

import java.util.Locale;


//TODO: CHANGE DRIVE TO USE PINPOINT FOR HEADING RATHER THAN INTERNAL IMU
public class Drive {
    public static DcMotorEx frontRight;
    public static DcMotorEx frontLeft;
    public static DcMotorEx backRight;
    public static DcMotorEx backLeft;

    public static double rx;

    public static GoBildaPinpointDriver odo;
    public double botHeading;

    private PIDFController turnController = new PIDFController(.04, 0, 0, 0); //TODO: Feedforward component?
    public double IMUOffset;

    public boolean RobotCentric;

    public Toggle slowmode;

    public double YTarget = 6;

//    public double turnP = 0, turnI = 0, turnD = 0, turnF = 0;

    public Drive(HardwareMap hardwareMap, boolean robotCentric, int offsetIMU) {
        slowmode = new Toggle(false);
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setOffsets(6.6147323, 3.30708661, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        this.RobotCentric = robotCentric;
        odo.setHeading(transitionHeading, AngleUnit.RADIANS);
        botHeading = odo.getHeading(AngleUnit.RADIANS) + IMUOffset;

        rx = 0;
        IMUOffset = offsetIMU;
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, double voltageMultiplier) {
        odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
        botHeading = odo.getHeading(AngleUnit.RADIANS) + IMUOffset;

        if (!RobotCentric) {
            //Field Centric Drive:
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

//            if (gamepad1.right_bumper) {
//                rx = calcRotBasedOnIdeal(Vision.tX, 0);
//            }
//            if (rx == 0){
//                rx = calcRotBasedOnIdeal(botHeading, targetHeading);
//            }
//            else {
//                targetHeading = botHeading;
//            }
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.2;  // Counteract imperfect strafing
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

//            slowmode.update(gamepad2.dpad_left);
//            if (slowmode.value() == true) {
//                denominator *= 2;
//            }

            double frontLeftPower = (rotY + rotX + rx) / denominator * voltageMultiplier;
            double backLeftPower = (rotY - rotX + rx) / denominator * voltageMultiplier;
            double frontRightPower = (rotY - rotX - rx) / denominator * voltageMultiplier;
            double backRightPower = (rotY + rotX - rx) / denominator * voltageMultiplier;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            //Reset IMU:
            if (gamepad1.share) {
                resetImu();
            }
        } else {
//            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.2; // Counteract imperfect strafing
            rx = (gamepad1.right_stick_x);

            if (gamepad1.right_bumper) {
                if (isRed && Transport.shooterVelocityTarget > 700) {
                    rx = calcRotBasedOnIdeal(Vision.tX, -2, voltageMultiplier);
                } else if (!isRed && Transport.shooterVelocityTarget > 700){
                    rx = calcRotBasedOnIdeal(Vision.tX, 2, voltageMultiplier);
                } else {
                    rx = calcRotBasedOnIdeal(Vision.tX, 0, voltageMultiplier);
                }
            }
            if (gamepad1.dpad_down) {
                rx = calcRotBasedOnIdeal(botHeading, 0, voltageMultiplier);
            }
//            if (rx == 0){
//                rx = calcRotBasedOnIdeal(botHeading, targetHeading);
//            }
//            else {
//                rx = gamepad1.right_stick_x;
//                targetHeading = botHeading;
//            }

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

//            slowmode.update(gamepad2.dpad_left);
//            if (slowmode.value() == true) {
//                denominator *= 2;
//            }


            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
        }
    }
    private double calcRotBasedOnIdeal(double heading, double idealHeading, double voltageMultiplier) {
        // Error in rotations (should always be between (-0.5,0.5))
        double err = angleWrap(idealHeading - heading);
        return turnController.calculate(err, 0) * voltageMultiplier;
    }
    public double angleWrap(double angle) {
        angle = Math.toRadians(angle);
        // Changes any angle between [-179,180] degrees
        // If rotation is greater than half a full rotation, it would be more efficient to turn the other way
        while (Math.abs(angle) > Math.PI) {
            angle -= 2 * Math.PI * (angle > 0 ? 1 : -1); // if angle > 0 * 1, < 0 * -1
        }
        return Math.toDegrees(angle);
    }

    public void resetImu () {
        IMUOffset = 0;
        odo.setHeading(0, AngleUnit.RADIANS);
    }
}
