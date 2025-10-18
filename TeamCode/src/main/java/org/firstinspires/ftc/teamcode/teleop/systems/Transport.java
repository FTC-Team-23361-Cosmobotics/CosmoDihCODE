package org.firstinspires.ftc.teamcode.teleop.systems;

import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.isAuto;
import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.isRed;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;


public class Transport {
    public Toggle intakeToggle, redToggle, manualMode;
    private ServoImplEx gate;
    public static DcMotorEx intake;

    public static MotorEx shooter;

    //Shooter PID
    private PIDFController shooterController;

    public final double shooterp = 0.00007, shooteri = .0000001, shooterd = .0000001, shooterf = .0006;
    public double shooterpid;

    //SERVO POSITIONS

    public double gatePos;

    //SERVO VALUES
    public static double openGate = .115;

    public static double closeGate = .163;
    //MOTOR POWER
    public static double intakePower, shooterPower;

    //MOTOR VELOCITY

    public static double shooterVelocity;

    //MOTOR TARGETS
    public static double shooterVelocityTarget;

    //ELAPSED TIMES
    ElapsedTime shootWait;

    //WAIT VALUES
    public static double fireWait = .5;

    public static double outtakeWait = .5;
    public static double sheathWait = 2.5;

    //USEFUL STATES

    public final double dormant = 0;
    public final double intaking = 1;

    public final double outtaking = -.35;

    public final double shootingLong = 1500;

    public final double shootingMed = 900;

    public final double shootingShort = 600;

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
    public static double fireTolerance = 120;
    public boolean inRange(double currentVelocity, double targetVelocity) {
        if (Math.abs(targetVelocity - currentVelocity) < fireTolerance) {
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

        shootWait = new ElapsedTime();
        shootWait.reset();

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterController = new PIDFController(shooterp, shooteri, shooterd, shooterf);
        shooter = new MotorEx(hardwareMap, "shooter", Motor.GoBILDA.RPM_1150);
        shooter.setRunMode(MotorEx.RunMode.RawPower);

        gate = hardwareMap.get(ServoImplEx.class, "gate");

        if (isAuto) { //TODO: Find Out How to Deal w Init: Remove All Mvmt in Init?
            intakePower = dormant;
            shooterVelocity = dormant;
            shooterVelocityTarget = dormant;
            gate.setPosition(closeGate);
        } else {
            intakePower = dormant;
            shooterVelocity = dormant;
            shooterVelocityTarget = dormant;
            gate.setPosition(closeGate);
        }
    }

    public void update() {
        intake.setPower(intakePower);

        shooterVelocity = shooter.getVelocity();
        shooterpid = shooterController.calculate(shooterVelocity, shooterVelocityTarget);
        shooter.set(shooterpid);

        gate.setPosition(gatePos);

        switch (ricoTransport) {
            case HOME:
                intakePower = dormant;
                shooterVelocityTarget = dormant;
                gatePos = closeGate;
                break;
            case INTAKE:
                intakePower = intaking;
                shooterVelocityTarget = dormant;
                gatePos = closeGate;
                break;
            case OUTTAKE:
                intakePower = outtaking;
                shooterVelocityTarget = dormant;
                gatePos = closeGate;
                break;
            case POWER_SHOOTER_SHORT:
                intakePower = dormant;
                shooterVelocityTarget = shootingShort;
                gatePos = closeGate;
                break;
            case POWER_SHOOTER_MED:
                intakePower = dormant;
                shooterVelocityTarget = shootingMed;
                gatePos = closeGate;
                break;
            case POWER_SHOOTER_LONG:
                intakePower = dormant;
                shooterVelocityTarget = shootingLong;
                gatePos = closeGate;
                break;
            case SHOOT:
                shooterVelocityTarget = intaking;
                gatePos = openGate;
                break;
            default:
                ricoTransport = RicoTransport.HOME;
        }
    }


    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        manualMode.update(gamepad2.back);

        intake.setPower(intakePower);

        shooterVelocity = shooter.getVelocity();
        shooterpid = shooterController.calculate(shooterVelocity, shooterVelocityTarget);
        shooter.set(shooterpid);

        gate.setPosition(gatePos);

        if (!manualMode.value()) {
            switch (ricoTransport) {
                case HOME:
                    intakePower = dormant;
                    shooterVelocityTarget = dormant;
                    gatePos = closeGate;
                    if (gamepad1.b) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.OUTTAKE;
                    }
                    if (gamepad1.x) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.POWER_SHOOTER_SHORT;
                    }
                    if (gamepad1.right_bumper) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.POWER_SHOOTER_MED;
                    }
                    if (gamepad1.y) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.POWER_SHOOTER_LONG;
                    }
                    break;
                case OUTTAKE:
                    intakePower = outtaking;
                    shooterVelocityTarget = dormant;
                    gatePos = closeGate;
                    if (shootWait.seconds() >= outtakeWait) {
                        ricoTransport = RicoTransport.HOME;
                    }
                    break;
                case POWER_SHOOTER_SHORT:
                    intakePower = intaking;
                    shooterVelocityTarget = shootingShort;
                    gatePos = closeGate;
                    if (gamepad1.left_bumper && inRange(shooterVelocityTarget, shooterVelocity)) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.SHOOT;
                    }
                    if (gamepad1.b) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.OUTTAKE;
                    }
                    break;
                case POWER_SHOOTER_MED:
                    intakePower = intaking;
                    shooterVelocityTarget = shootingMed;
                    gatePos = closeGate;
                    if (gamepad1.left_bumper && inRange(shooterVelocityTarget, shooterVelocity)) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.SHOOT;
                    }
                    if (gamepad1.b) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.OUTTAKE;
                    }
                    break;
                case POWER_SHOOTER_LONG:
                    intakePower = intaking;
                    shooterVelocityTarget = shootingLong;
                    gatePos = closeGate;
                    if (gamepad1.left_bumper && inRange(shooterVelocityTarget, shooterVelocity)) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.SHOOT;
                    }
                    if (gamepad1.b) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.OUTTAKE;
                    }
                    break;
                case SHOOT:
                    gatePos = openGate;
                    break;
                default:
                    ricoTransport = RicoTransport.HOME;


            }

            if (gamepad1.a && ricoTransport != ricoTransport.HOME) {
                ricoTransport = ricoTransport.HOME;
                intakeToggle.value = false;
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
