package org.firstinspires.ftc.teamcode.teleop.systems;

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
    public ElapsedTime shootWait;
    //Hardware
    public RevColorSensorV3 lowSensor;
    public Toggle intakeToggle, redToggle, manualMode, hapticFeedback;
    public static DcMotorEx transfer;
    public static DcMotorEx intake;

    public static MotorEx shooter;
    //Limelight Stuff
    public double limelightMountAngleDegrees = 0;
    // distance from the center of the Limelight lens to the floor
    public double limelightLensHeightInches = 17.1653996;
    // distance from the target to the floor
    public double goalHeightInches = 29.5;

    //Color Sensor Values and Logic
    public static int lowLevel = 0;


    //Shooter PID
    private PIDFController shooterController;

    public double shooterp = 0.01, shooteri = 0, shooterd = 0, shooterf = .00007;
    public double shooterpid;

    //SERVO POSITIONS

    public double transferPower;

    //MOTOR POWER
    public static double intakePower;

    //MOTOR VELOCITY

    public static double shooterVelocity;

    //MOTOR TARGETS
    public static double shooterVelocityTarget;

    //ELAPSED TIMES
    public static ElapsedTime matchTimer;

    //USEFUL STATES

    public final double dormant = 0;

    public final double transferring = .75;
    public final double intaking = 1;

    public final double outtaking = -1;

    public final double shootingLong = 750;

    public final double shootingMed = 590;

    public final double shootingShort = 520;

    public final double emergencyEject = 150;

    //FSMs:
    public enum RicoTransport {
        HOME,
        INTAKE,
        OUTTAKE,
        POWER_SHOOTER_SHORT,

        POWER_SHOOTER_MED,

        POWER_SHOOTER_LONG,

        AUTO_POWER,
        SHOOT

    }

    public RicoTransport ricoTransport = RicoTransport.HOME;
    public RicoTransport returnCase = RicoTransport.HOME;

    //Fire Ready?
    public static double fireTolerance = 40;
    public boolean inRange(double currentVelocity, double targetVelocity) {
        if (currentVelocity > (targetVelocity - fireTolerance) ) {
            return true;
        } else {
            return false;
        }
    }

    public double calcVelocity() {
        double targetOffsetAngle_Vertical = Vision.tY;
        double angleToGoal = Math.toRadians(limelightMountAngleDegrees + targetOffsetAngle_Vertical);
        //calculate distance
        double distanceFromLimelightToTagInches = ((goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoal));
        //TODO: TUNE
        double scale = 460;
        double velocity = scale + (distanceFromLimelightToTagInches * 1.5);
        double interVelocity = (int) (velocity / 10);
        double velocityTar = interVelocity * 10;
        return velocityTar;
    }

    public Transport(HardwareMap hardwareMap) {
        intakeToggle = new Toggle(false);
        manualMode = new Toggle(false);

        if (isRed) {
            redToggle = new Toggle(true);
        } else {
            redToggle = new Toggle(false);
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

        lowSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        lowSensor.enableLed(true);

        if (inAuto) {
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
        if (shooterVelocityTarget == 0) {
            shooterp = 0;
            shooterf = 0;
        } else {
            shooterp = .02;
            shooterf = .00007;
        }

        intake.setPower(intakePower);

        shooterVelocity = shooter.getVelocity();
        shooterpid = shooterController.calculate(shooterVelocity, shooterVelocityTarget) * voltageMultiplier;
        shooter.set(shooterpid);

        transfer.setPower(transferPower);

        fireTolerance = 150;
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
            case INTAKE:
                intakePower = intaking;
                shooterVelocityTarget = dormant;
                transferPower = outtaking * .25 * voltageMultiplier;
                break;
            case POWER_SHOOTER_SHORT:
                intakePower = dormant;
                shooterVelocityTarget = shootingShort;
                transferPower = dormant;
                fireTolerance = 40;
                break;
            case POWER_SHOOTER_MED:
                intakePower = dormant;
                shooterVelocityTarget = shootingMed;
                transferPower = dormant;
                fireTolerance = 40;
                break;
            case POWER_SHOOTER_LONG:
                intakePower = dormant;
                shooterVelocityTarget = shootingLong;
                transferPower = dormant;
                fireTolerance = 40;
                break;
            case SHOOT:
                if (shooterVelocityTarget <= 550) {
                    fireTolerance = 150;
                } else if (shooterVelocityTarget <= 660) {
                    fireTolerance = 150;
                } else {
                    fireTolerance = 60;
                }
                if (shooterVelocityTarget >= 660 || shooterVelocityTarget <= 540) {
                    transferPower = intaking;
                    intakePower = intaking;
                } else {
                    transferPower = transferring * voltageMultiplier;
                    intakePower = transferring * voltageMultiplier;
                }
                break;
            default:
                ricoTransport = RicoTransport.HOME;
        }
    }


    public void update(Gamepad gamepad1, Gamepad gamepad2, double voltageMultiplier) {
//        manualMode.update(gamepad2.back);
        if (shooterVelocityTarget == 0) {
            shooterp = 0;
            shooterf = 0;
        } else {
            shooterp = .02;
            shooterf = .00007;
        }

        intake.setPower(intakePower);

        shooterVelocity = shooter.getVelocity();
        shooterpid = shooterController.calculate(shooterVelocity, shooterVelocityTarget) * voltageMultiplier;
        shooter.set(shooterpid);

        transfer.setPower(transferPower);

//        if (!manualMode.value()) {
            switch (ricoTransport) {
                case HOME:
                    intakePower = dormant;
                    shooterVelocityTarget = dormant;
                    transferPower = dormant;
                    if (gamepad1.left_bumper) {
                        ricoTransport = RicoTransport.INTAKE;
                    }
                    if (gamepad1.left_trigger > 0) {
                        ricoTransport = RicoTransport.POWER_SHOOTER_SHORT;
                    }
                    if (gamepad1.right_trigger > 0) {
                        ricoTransport = RicoTransport.AUTO_POWER;
                    }
                    if (gamepad1.triangle) {
                        ricoTransport = RicoTransport.POWER_SHOOTER_MED;
                    }
                    if (gamepad1.circle) {
                        returnCase = RicoTransport.HOME;
                        ricoTransport = RicoTransport.OUTTAKE;
                    }
                    if (gamepad1.dpad_up) {
                        ricoTransport = RicoTransport.POWER_SHOOTER_LONG;
                    }
                    break;
                case INTAKE:
                    lowLevel = lowSensor.argb();
                    if (lowLevel < 0) {
                        gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
                    } else {
                        gamepad1.stopRumble();
                    }
                    intakePower = intaking;
                    shooterVelocityTarget = dormant;
                    transferPower = outtaking * .25 * voltageMultiplier;
                    if (gamepad1.left_trigger > 0) {
                        gamepad1.stopRumble();
                        ricoTransport = RicoTransport.POWER_SHOOTER_SHORT;
                    }
                    if (gamepad1.right_trigger > 0) {
                        gamepad1.stopRumble();
                        ricoTransport = RicoTransport.AUTO_POWER;
                    }
                    if (gamepad1.triangle) {
                        gamepad1.stopRumble();
                        ricoTransport = RicoTransport.POWER_SHOOTER_MED;
                    }
                    if (gamepad1.circle) {
                        gamepad1.stopRumble();
                        returnCase = RicoTransport.INTAKE;
                        ricoTransport = RicoTransport.OUTTAKE;
                    }
                    if (gamepad1.dpad_up) {
                        gamepad1.stopRumble();
                        ricoTransport = RicoTransport.POWER_SHOOTER_LONG;
                    }
                    break;
                case OUTTAKE:
                    intakePower = outtaking;
                    transferPower = outtaking;
                    if (!gamepad1.circle) {
                        ricoTransport = returnCase;
                    }
                    break;
                case POWER_SHOOTER_SHORT:
                    intakePower = dormant;
                    shooterVelocityTarget = shootingShort;
                    transferPower = dormant;
                    fireTolerance = 40;
                    if (gamepad1.left_bumper) {
                        ricoTransport = RicoTransport.INTAKE;
                    }
                    if ((gamepad1.right_bumper || gamepad1.dpad_right) && inRange(shooterVelocity, shooterVelocityTarget)) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.SHOOT;
                    }
                    if (gamepad1.right_trigger > 0) {
                        ricoTransport = RicoTransport.AUTO_POWER;
                    }
                    if (gamepad1.triangle) {
                        ricoTransport = RicoTransport.POWER_SHOOTER_MED;
                    }
                    if (gamepad1.dpad_up) {
                        ricoTransport = RicoTransport.POWER_SHOOTER_LONG;
                    }
                    if (gamepad1.circle) {
                        returnCase = RicoTransport.POWER_SHOOTER_SHORT;
                        ricoTransport = RicoTransport.OUTTAKE;
                    }
                    break;
                case POWER_SHOOTER_MED:
                    intakePower = dormant;
                    shooterVelocityTarget = shootingMed;
                    transferPower = dormant;
                    fireTolerance = 40;
                    if (gamepad1.left_bumper) {
                        ricoTransport = RicoTransport.INTAKE;
                    }
                    if ((gamepad1.right_bumper || gamepad1.dpad_right) && inRange(shooterVelocity, shooterVelocityTarget)) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.SHOOT;
                    }
                    if (gamepad1.left_trigger > 0) {
                        ricoTransport = RicoTransport.POWER_SHOOTER_SHORT;
                    }
                    if (gamepad1.right_trigger > 0) {
                        ricoTransport = RicoTransport.AUTO_POWER;
                    }
                    if (gamepad1.dpad_up) {
                        ricoTransport = RicoTransport.POWER_SHOOTER_LONG;
                    }
                    if (gamepad1.circle) {
                        returnCase = RicoTransport.POWER_SHOOTER_MED;
                        ricoTransport = RicoTransport.OUTTAKE;
                    }
                    break;
                case POWER_SHOOTER_LONG:
                    intakePower = dormant;
                    shooterVelocityTarget = shootingLong;
                    transferPower = dormant;
                    fireTolerance = 40;
                    if (gamepad1.left_bumper) {
                        ricoTransport = RicoTransport.INTAKE;
                    }
                    if ((gamepad1.right_bumper || gamepad1.dpad_right) && inRange(shooterVelocity, shooterVelocityTarget)) {
                        shootWait.reset();
                        ricoTransport = RicoTransport.SHOOT;
                    }
                    if (gamepad1.right_trigger > 0) {
                        ricoTransport = RicoTransport.AUTO_POWER;
                    }
                    if (gamepad1.left_trigger > 0) {
                        ricoTransport = RicoTransport.POWER_SHOOTER_SHORT;
                    }
                    if (gamepad1.triangle) {
                        ricoTransport = RicoTransport.POWER_SHOOTER_MED;
                    }
                    if (gamepad1.circle) {
                        returnCase = RicoTransport.POWER_SHOOTER_LONG;
                        ricoTransport = RicoTransport.OUTTAKE;
                    }
                    break;
                case AUTO_POWER:
                    intakePower = dormant;
                    if (calcVelocity() < 860)
                        shooterVelocityTarget = calcVelocity();
                    else {
                        shooterVelocityTarget = 860;
                    }

                    transferPower = dormant;
                    fireTolerance = 40;
                    if (gamepad1.left_bumper) {
                        ricoTransport = RicoTransport.INTAKE;
                    }
                    if ((gamepad1.right_bumper || gamepad1.dpad_right) && inRange(shooterVelocity, shooterVelocityTarget)){
                        shootWait.reset();
                        ricoTransport = RicoTransport.SHOOT;
                    }
                    if (gamepad1.left_trigger > 0) {
                        ricoTransport = RicoTransport.POWER_SHOOTER_SHORT;
                    }
                    if (gamepad1.triangle) {
                        ricoTransport = RicoTransport.POWER_SHOOTER_MED;
                    }
                    if (gamepad1.circle) {
                        returnCase = RicoTransport.AUTO_POWER;
                        ricoTransport = RicoTransport.OUTTAKE;
                    }
                    if (gamepad1.dpad_up) {
                        ricoTransport = RicoTransport.POWER_SHOOTER_LONG;
                    }
                    break;
                case SHOOT:
                    //TODO: SHOOT WAIT DUE TO PID ERROR IN DT
                    if (shooterVelocityTarget <= 550) {
                        fireTolerance = 150;
                    } else if (shooterVelocityTarget <= 660) {
                        fireTolerance = 150;
                    } else {
                        fireTolerance = 60;
                    }
                    if (gamepad1.left_bumper) {
                        ricoTransport = RicoTransport.INTAKE;
                    }
                    if (gamepad1.circle) {
                        returnCase = RicoTransport.SHOOT;
                        ricoTransport = RicoTransport.OUTTAKE;
                    }
                    if ((Math.abs(Drive.rx) < .15) && (inRange(shooterVelocity, shooterVelocityTarget) || gamepad1.dpad_left)) {
                        if (shooterVelocityTarget >= 660 || shooterVelocityTarget <= 540) {
                            transferPower = intaking;
                            intakePower = intaking;
                        } else {
                            transferPower = transferring * voltageMultiplier;
                            intakePower = transferring * voltageMultiplier;
                        }
                    } else {
                        if (shooterVelocityTarget >= 660) {
                            transferPower = outtaking * .25 * voltageMultiplier;
                        } else {
                            transferPower = dormant;
                        }
                        intakePower = dormant;
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

//        } else {
////            if (gamepad2.b) {
////                intakePower = outtaking;
////            } else if (gamepad2.a) {
////                intakePower = intaking;
////            } else {
////                intakePower = dormant;
////            }
//        }

    }
}
