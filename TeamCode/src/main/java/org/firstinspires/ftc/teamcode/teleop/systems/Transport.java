package org.firstinspires.ftc.teamcode.teleop.systems;

import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.hapticOn;
import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.inAuto;
import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.isRed;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;


public class Transport {
    //Hardware
    public RevColorSensorV3 lowSensor;
    public Toggle intakeToggle, redToggle, manualMode, hapticFeedback;
    public static DcMotorEx transfer;
    public static DcMotorEx intake;

    public static MotorEx shooter;

    //Color Sensor Values and Logic
    public static int lowLevel = 0;

    public static int greenLowerBound = 0; //TODO: TUNE
    public static int greenUpperBound = 0;
    public static int purpleLowerBound = 0;
    public static int purpleUpperBound = 0;

    private final int green = 2;
    private final int purple = 1;
    private final int empty = 0;

    public int hasArtifact(int sensorValue) {
        if (sensorValue > greenLowerBound && sensorValue < greenUpperBound) {
            return green;
        } else if (sensorValue > purpleLowerBound && sensorValue < purpleUpperBound) {
            return purple;
        }
        return empty;
    }

    //Shooter PID
    private PIDFController shooterController;

    public final double shooterp = 0.01, shooteri = .0000001, shooterd = .00001, shooterf = .0007;
    public double shooterpid;

    //SERVO POSITIONS

    public double transferPower;

    //SERVO VALUES
    public static double openGate = .115;

    public static double closeGate = .163;
    //MOTOR POWER
    public static double intakePower;

    //MOTOR VELOCITY

    public static double shooterVelocity;

    //MOTOR TARGETS
    public static double shooterVelocityTarget;

    //ELAPSED TIMES
    public static ElapsedTime shootWait;
    public static ElapsedTime matchTimer;

    //WAIT VALUES

    public static double parkTime = 110;
    public static double fireWait = .5;

    public static double outtakeWait = .05;
    public static double sheathWait = 2.5;

    //RUMBLE DURATION (MS)
    public static int parkRumble = 1000;

    public static int intakeReadyRumble = 100;

    //USEFUL STATES

    public final double dormant = 0;

    public final double transferring = .7;
    public final double intaking = 1;

    public final double outtaking = -1;

    public final double shootingLong = 870;

    public final double shootingMed = 590;

    public final double shootingShort = 500;

    //FSMs:
    public enum RicoTransport {
        HOME,
        INTAKE,
        OUTTAKE,
        POWER_SHOOTER_SHORT,

        POWER_SHOOTER_MED,

        POWER_SHOOTER_LONG,
        SHOOT

    }

    public RicoTransport ricoTransport = RicoTransport.HOME;

    //Fire Ready?
    public static double fireTolerance = 150;
    public boolean inRange(double currentVelocity, double targetVelocity) {
        if (currentVelocity > (targetVelocity - fireTolerance) ) {
            return true;
        } else {
            return false;
        }
    }



    public Transport(HardwareMap hardwareMap) {
        intakeToggle = new Toggle(false);
        manualMode = new Toggle(false);

        if (isRed) {
            redToggle = new Toggle(true);
        } else {
            redToggle = new Toggle(false);
        }

        if (hapticOn) {
            hapticFeedback = new Toggle(true);
        } else {
            hapticFeedback = new Toggle(false);
        }

        matchTimer = new ElapsedTime();
        matchTimer.reset();

        shootWait = new ElapsedTime();
        shootWait.reset();

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterController = new PIDFController(shooterp, shooteri, shooterd, shooterf);
        shooter = new MotorEx(hardwareMap, "shooter", Motor.GoBILDA.RPM_1150);
        shooter.setRunMode(MotorEx.RunMode.RawPower);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        lowSensor = hardwareMap.get(RevColorSensorV3.class, "lowSensor");
        lowSensor.enableLed(true);

        if (inAuto) { //TODO: Find Out How to Deal w Init: Remove All Mvmt in Init?
            intakePower = dormant;
            shooterVelocity = dormant;
            shooterVelocityTarget = dormant;
            transfer.setPower(dormant);
        } else {
            intakePower = dormant;
            shooterVelocity = dormant;
            shooterVelocityTarget = dormant;
            transfer.setPower(dormant);
        }
    }

    public void update(double voltageMultiplier) {
//        lowLevel = lowSensor.argb();

        intake.setPower(intakePower * voltageMultiplier);

        shooterVelocity = shooter.getVelocity();
        shooterpid = shooterController.calculate(shooterVelocity, shooterVelocityTarget) * voltageMultiplier;
        shooter.set(shooterpid);

        transfer.setPower(transferPower * voltageMultiplier);

        switch (ricoTransport) {
            case HOME:
                intakePower = dormant;
                shooterVelocityTarget = dormant;
                transferPower = dormant;
                break;
            case OUTTAKE:
                intakePower = outtaking;
                shooterVelocityTarget = dormant;
                transferPower = outtaking;
                break;
            case POWER_SHOOTER_SHORT:
                intakePower = intaking;
                shooterVelocityTarget = shootingShort;
                transferPower = outtaking;
                break;
            case POWER_SHOOTER_MED:
                intakePower = intaking;
                shooterVelocityTarget = shootingMed;
                transferPower = outtaking;
                break;
            case POWER_SHOOTER_LONG:
                intakePower = intaking;
                shooterVelocityTarget = shootingLong;
                transferPower = outtaking;
                break;
            case SHOOT:
                intakePower = intaking;
                transferPower = transferring;
                break;
            default:
                ricoTransport = RicoTransport.HOME;
        }
    }


    public void update(Gamepad gamepad1, Gamepad gamepad2, double voltageMultiplier) {
        if (matchTimer.seconds() == parkTime) {
            gamepad1.rumble(parkRumble);
        }

        manualMode.update(gamepad2.back);
        hapticFeedback.update(gamepad1.dpad_right);

        intake.setPower(intakePower * voltageMultiplier);

        shooterVelocity = shooter.getVelocity();
        shooterpid = shooterController.calculate(shooterVelocity, shooterVelocityTarget) * voltageMultiplier;
        shooter.set(shooterpid);

        transfer.setPower(transferPower * voltageMultiplier);

        if (!manualMode.value()) {
            switch (ricoTransport) {
                case HOME:
                    intakePower = dormant;
                    shooterVelocityTarget = dormant;
                    transferPower = dormant;
                    if (gamepad1.circle) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.OUTTAKE;
                    }
                    if (gamepad1.cross) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.POWER_SHOOTER_SHORT;
                    }
                    if (gamepad1.left_bumper) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.POWER_SHOOTER_MED;
                    }
                    if (gamepad1.dpad_up) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.POWER_SHOOTER_LONG;
                    }
                    break;
                case OUTTAKE:
                    intakePower = outtaking;
                    transferPower = outtaking;
                    if (!gamepad1.circle) {
                        ricoTransport = RicoTransport.HOME;
                    }
                    break;
                case POWER_SHOOTER_SHORT:
                    lowLevel = lowSensor.argb();
                    intakePower = intaking;
                    shooterVelocityTarget = shootingShort;
                    transferPower = outtaking / 2;
                    fireTolerance = 150;
                    if (gamepad1.right_bumper && inRange(shooterVelocity, shooterVelocityTarget)) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.SHOOT;
                    }
                    if (gamepad1.left_bumper) {
                        ricoTransport = RicoTransport.POWER_SHOOTER_MED;
                    }
                    if (gamepad1.dpad_up) {
                        ricoTransport = RicoTransport.POWER_SHOOTER_LONG;
                    }
                    if (gamepad1.circle) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.OUTTAKE;
                    }
                    break;
                case POWER_SHOOTER_MED:
                    intakePower = intaking;
                    shooterVelocityTarget = shootingMed;
                    transferPower = outtaking / 2;
                    fireTolerance = 70;
                    if (gamepad1.right_bumper && inRange(shooterVelocity, shooterVelocityTarget)) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.SHOOT;
                    }
                    if (gamepad1.cross) {
                        ricoTransport = RicoTransport.POWER_SHOOTER_SHORT;
                    }
                    if (gamepad1.dpad_up) {
                        ricoTransport = RicoTransport.POWER_SHOOTER_LONG;
                    }
                    if (gamepad1.circle) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.OUTTAKE;
                    }
                    break;
                case POWER_SHOOTER_LONG:
                    intakePower = dormant;
                    shooterVelocityTarget = shootingLong;
                    transferPower = dormant;
                    fireTolerance = 30;
                    if (gamepad1.right_bumper && inRange(shooterVelocity, shooterVelocityTarget)) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.SHOOT;
                    }
                    if (gamepad1.cross) {
                        ricoTransport = RicoTransport.POWER_SHOOTER_SHORT;
                    }
                    if (gamepad1.left_bumper) {
                        ricoTransport = RicoTransport.POWER_SHOOTER_MED;
                    }
                    if (gamepad1.circle) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.OUTTAKE;
                    }
                    break;
                case SHOOT:
                    if (inRange(shooterVelocity, shooterVelocityTarget)) {
                        transferPower = transferring;
                        intakePower = intaking;
                    } else {
                        transferPower = outtaking;
                    }
                    break;
                default:
                    ricoTransport = RicoTransport.HOME;


            }

            if (gamepad1.square && ricoTransport != RicoTransport.HOME) {
                ricoTransport = RicoTransport.HOME;
                intakeToggle.value = false;
            }

            if (gamepad2.x && ricoTransport != RicoTransport.SHOOT) {
                ricoTransport = RicoTransport.SHOOT;
            }

        } else {
            if (gamepad2.b) {
                intakePower = outtaking;
            } else if (gamepad2.a) {
                intakePower = intaking;
            } else {
                intakePower = dormant;
            }
        }

    }
}
