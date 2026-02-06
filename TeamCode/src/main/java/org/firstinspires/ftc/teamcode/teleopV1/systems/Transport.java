package org.firstinspires.ftc.teamcode.teleopV1.systems;

import static org.firstinspires.ftc.teamcode.teleopV1.utils.GlobalVars.inAuto;
import static org.firstinspires.ftc.teamcode.teleopV1.utils.GlobalVars.isRed;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teleopV1.utils.Toggle;

public class Transport {
    public ElapsedTime shootWait;
    //Hardware
//    public RevColorSensorV3 lowSensor;
    public Toggle intakeToggle, redToggle, manualMode;
    public static DcMotorEx transfer, intake;
    public static MotorEx shooter;
    public static Servo led;
    //Limelight Stuff
    public double limelightMountAngleDegrees = 10;
    // distance from the center of the Limelight lens to the floor
    public double limelightLensHeightInches = 16.930377953;
    // distance from the target to the floor
    public double goalHeightInches = 29.5;

    //Color Sensor Values and Logic
    public static int lowLevel = 0;


    //Shooter PID
    private PIDFController shooterController;

    public double shooterp = 0.01, shooteri = 0, shooterd = 0, kV = .000400186335403727, kS = .001;
    public double shooterpid;
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
    public final double transferring = 1;
    public final double intaking = 1;
    public final double outtaking = -1;
    public final double shootingLong = 800;
    public final double shootingMed = 600;
    public final double shootingShort = 510;
    public final double emergencyEject = -300;
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
        if (Math.abs(targetVelocity - currentVelocity) < fireTolerance) {
            return true;
        } else {
            return false;
        }
    }

    public double calcVelocity() {
        double angleToGoal = Math.toRadians(limelightMountAngleDegrees + Vision.tY);
        double distanceFromLimelightToTagInches = ((goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoal));
        double intercept = 450;
        double velocity = intercept + (distanceFromLimelightToTagInches * 1.62);
        double interVelocity = (int) (velocity / 10);
        double velocityTarget = interVelocity * 10;
        return velocityTarget;
    }

    public double calcVelocityLong() {
        double angleToGoal = Math.toRadians(limelightMountAngleDegrees + Vision.tY);
        double distanceFromLimelightToTagInches = ((goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoal));
        double intercept = 560;
        double velocity = intercept + (distanceFromLimelightToTagInches * 1.56);
        double interVelocity = (int) (velocity / 10);
        double velocityTarget = interVelocity * 10;
        return velocityTarget;
    }

    public Transport(HardwareMap hardwareMap) {
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
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterController = new PIDFController(shooterp, shooteri, shooterd, kV);
        shooter = new MotorEx(hardwareMap, "shooter", Motor.GoBILDA.BARE);
        shooter.setRunMode(MotorEx.RunMode.RawPower);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lowSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
//        lowSensor.enableLed(true);

        led = hardwareMap.get(ServoImplEx.class, "led");

        if (inAuto) {
            intakePower = dormant;
            shooterVelocity = dormant;
            shooterVelocityTarget = dormant;
            transfer.setPower(dormant);
            led.setPosition(0);
        } else {
            intakePower = dormant;
            shooterVelocity = dormant;
            shooterVelocityTarget = dormant;
            transfer.setPower(dormant);
            led.setPosition(.6);
        }
    }

    public void update(double voltageMultiplier) {
        if (shooterVelocityTarget == 0) {
            shooterp = 0;
            kV = 0;
        } else {
            shooterp = .025;
            kV = .000400186335403727;
        }

        intake.setPower(intakePower);

        shooterVelocity = shooter.getVelocity();
        shooterController.setP(shooterp);
        shooterController.setF(kV * voltageMultiplier);
        shooterpid = shooterController.calculate(shooterVelocity, shooterVelocityTarget) + Math.abs((Math.signum(shooterVelocityTarget) * kS * voltageMultiplier));
        shooter.set(Range.clip(shooterpid, -1, 1));

        transfer.setPower(transferPower);

        switch (ricoTransport) {
            case HOME:
                intakePower = dormant;
                shooterVelocityTarget = dormant;
                transferPower = dormant;
                break;
            case OUTTAKE:
                intakePower = outtaking;
                shooterVelocityTarget = emergencyEject;
                transferPower = outtaking;
                break;
            case INTAKE:
                intakePower = intaking;
                shooterVelocityTarget = dormant;
                transferPower = outtaking * .25;
                break;
            case POWER_SHOOTER_SHORT:
                intakePower = 1;
                shooterVelocityTarget = 560;
                transferPower = -.25;
                fireTolerance = 30;
                break;
            case POWER_SHOOTER_MED:
                intakePower = dormant;
                shooterVelocityTarget = shootingMed;
                transferPower = dormant;
                fireTolerance = 30;
                break;
            case POWER_SHOOTER_LONG:
                intakePower = dormant;
                shooterVelocityTarget = shootingLong;
                transferPower = dormant;
                fireTolerance = 30;
                break;
            case SHOOT:
                if (shooterVelocityTarget <= 550) {
                    fireTolerance = 150;
                } else if (shooterVelocityTarget <= 660) {
                    fireTolerance = 150;
                } else {
                    fireTolerance = 100;
                }
                if (shooterVelocityTarget >= 660 || shooterVelocityTarget <= 540) {
                    transferPower = intaking;
                    intakePower = intaking;
                } else {
                    transferPower = intaking;
                    intakePower = transferring;
                }
                break;
            default:
                ricoTransport = RicoTransport.HOME;
        }
    }


    public void update(Gamepad gamepad1, Gamepad gamepad2, double voltageMultiplier) {
        if (Vision.tX < 2 && Vision.tX > -2 && Vision.validResult) {
            led.setPosition(.5);
        } else if (!isRed && Vision.tX < 6 && Vision.tX > 2.5 && Vision.validResult) {
            led.setPosition(.63);
        } else if (isRed && Vision.tX > -6 && Vision.tX < -2.5 && Vision.validResult) {
            led.setPosition(.63);
        } else if (!Vision.isConnected){
            led.setPosition(0);
        } else {
            led.setPosition(.3);
        }

        if (shooterVelocityTarget == 0) {
            shooterp = 0;
            kV = 0;
        } else {
            shooterp = .025;
            kV = .000400186335403727;
        }

        intake.setPower(intakePower);

        shooterVelocity = shooter.getVelocity();
        shooterController.setP(shooterp);
        shooterController.setF(kV * voltageMultiplier);
        shooterpid = shooterController.calculate(shooterVelocity, shooterVelocityTarget) + Math.abs((Math.signum(shooterVelocityTarget) * kS * voltageMultiplier));
        shooter.set(Range.clip(shooterpid, -1, 1));

        transfer.setPower(transferPower);
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
//                    lowLevel = lowSensor.argb();
//                    if (lowLevel < 0) {
//                        gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
//                    } else {
//                        gamepad1.stopRumble();
//                    }
                    intakePower = intaking;
                    shooterVelocityTarget = dormant;
                    transferPower = outtaking * .25;
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
                    shooterVelocityTarget = emergencyEject;
                    if (!gamepad1.circle) {
                        ricoTransport = returnCase;
                    }
                    break;
                case POWER_SHOOTER_SHORT:
                    intakePower = dormant;
                    shooterVelocityTarget = shootingShort;
                    transferPower = dormant;
                    fireTolerance = 30;
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
                    fireTolerance = 30;
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
                    fireTolerance = 30;
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
                        if (Vision.tY <= -6) {
                            shooterVelocityTarget = calcVelocityLong();
                        } else {
                            shooterVelocityTarget = calcVelocity();
                        }
                    else {
                        shooterVelocityTarget = 860;
                    }
                    transferPower = dormant;
                    fireTolerance = 30;
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
                    if (shooterVelocityTarget <= 550) {
                        fireTolerance = 150;
                    } else if (shooterVelocityTarget <= 660) {
                        fireTolerance = 150;
                    } else {
                        fireTolerance = 100;
                    }
                    if (gamepad1.left_bumper) {
                        ricoTransport = RicoTransport.INTAKE;
                    }
                    if (gamepad1.circle) {
                        returnCase = RicoTransport.SHOOT;
                        ricoTransport = RicoTransport.OUTTAKE;
                    }
                    if ((Math.abs(Drive.rx) < .1) && (inRange(shooterVelocity, shooterVelocityTarget) || gamepad1.dpad_left)) {
                        if (shooterVelocityTarget <= 660 && shooterVelocityTarget >= 560) {
                            transferPower = intaking;
                            intakePower = intaking;
                        } else {
                            transferPower = intaking;
                            intakePower = intaking;
                        }
                    } else {
                        if (shooterVelocityTarget >= 660) {
                            transferPower = dormant; //TODO: maybe dormant?
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
//                intakeToggle.value = false;
            }
    }
}
