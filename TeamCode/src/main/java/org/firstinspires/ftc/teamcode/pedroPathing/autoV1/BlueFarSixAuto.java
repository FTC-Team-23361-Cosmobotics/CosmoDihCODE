package org.firstinspires.ftc.teamcode.pedroPathing.autoV1;

import static org.firstinspires.ftc.teamcode.teleopV1.utils.GlobalVars.transitionHeading;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsV1;

import org.firstinspires.ftc.teamcode.teleopV1.systems.Transport;
import org.firstinspires.ftc.teamcode.teleopV1.utils.GlobalVars;

@Autonomous(name = "BlueFarAuto: Six Artifacts", group = "FarAuto", preselectTeleOp = "BlueDihCodeTeleop")
public class BlueFarSixAuto extends OpMode {
    private VoltageSensor voltageSensor;
    private Follower follower;

    private Transport transport;
    private Timer pathTimer, opmodeTimer;
    private int pathState;


    /** POSE COORDINATES **/
    public static double zoneX = 136, zoneY = 8, zoneHeading = Math.toRadians(180);
    public static double spikeX = 11, lowSpikeY = 42;
    public static double startX = 56, startY = 8, startHeading = Math.toRadians(90);
    public static double scoreX = 59, scoreY = 24, scoreHeading = Math.toRadians(115);

    public static double parkX = 36, parkY = 14, parkHeading = Math.toRadians(0);

    /** START, SCORE, AND PARK POSES **/
    private final Pose startPose = new Pose(startX, startY, startHeading); // Start Pose of our robot.
    private final Pose scorePose = new Pose(scoreX, scoreY, scoreHeading); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose parkPose = new Pose(parkX, parkY, parkHeading);


    /** SPIKE CONTROL POINTS **/

    private final Pose lowSpikeControlPtPose = new Pose(70, 39);

    /** SPIKE POSES **/
    private final Pose lowSpikePose = new Pose(spikeX, lowSpikeY, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose zonePose = new Pose(zoneX, zoneY, zoneHeading);// Lowest (Third Set) of Artifacts from the Spike Mark.

    private PathChain scorePreload, grabGPP, scoreGPP, grabZone, scoreZone, park;
    public void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startHeading, scoreHeading)
                .build();

        grabGPP = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, lowSpikeControlPtPose, lowSpikePose))
                .setLinearHeadingInterpolation(scoreHeading, 0)
                .build();

        scoreGPP = follower.pathBuilder()
                .addPath(new BezierCurve(lowSpikePose, lowSpikeControlPtPose, scorePose))
                .setLinearHeadingInterpolation(0, scoreHeading)
                .build();
        scoreZone = follower.pathBuilder()
                .addPath(new BezierLine(zonePose, scorePose))
                .setLinearHeadingInterpolation(zoneHeading, scoreHeading)
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, parkPose))
                .setLinearHeadingInterpolation(scoreHeading, parkHeading)
                .build();
    }

    private final double shootWaitPathOne = 2;


    private final double shootWaitPathTwo = 2;
    private final double intakeWait = .1;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_LONG;
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 1 && transport.inRange(Transport.shooterVelocity, Transport.shooterVelocityTarget)) {
                    transport.ricoTransport = Transport.RicoTransport.SHOOT;
                } else {
                    transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_LONG;
                }
                if (pathTimer.getElapsedTimeSeconds() > (shootWaitPathOne * 12 / voltageSensor.getVoltage())) {
                    transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_LONG;
                    follower.followPath(grabGPP,true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTimeSeconds() > intakeWait) {
                    follower.followPath(scoreGPP,true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 1 && transport.inRange(Transport.shooterVelocity, Transport.shooterVelocityTarget)) {
                    transport.ricoTransport = Transport.RicoTransport.SHOOT;
                } else {
                    transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_LONG;
                }
                if (pathTimer.getElapsedTimeSeconds() > (shootWaitPathTwo * 12 / voltageSensor.getVoltage())) {
                    transport.ricoTransport = Transport.RicoTransport.HOME;
                    follower.followPath(grabZone);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > intakeWait) {
                    follower.followPath(scoreZone,true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTimeSeconds() > 1 && transport.inRange(Transport.shooterVelocity, Transport.shooterVelocityTarget)) {
                    transport.ricoTransport = Transport.RicoTransport.SHOOT;
                } else {
                    transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_LONG;
                }
                if (pathTimer.getElapsedTimeSeconds() > (shootWaitPathTwo * 12 / voltageSensor.getVoltage())) {
                    transport.ricoTransport = Transport.RicoTransport.HOME;
                    follower.followPath(park);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    transitionHeading = follower.getHeading();
                    setPathState(-1);
                }
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        GlobalVars.isRed = true;
        GlobalVars.inAuto = true;
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        transport = new Transport(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = ConstantsV1.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        //TODO: APRIL TAG VISION TELEMETRY GOES HERE
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        transport.update(14 / voltageSensor.getVoltage());
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Flywheel Velocity", Transport.shooterVelocity);
        telemetry.addData("Flywheel Target Velocity", Transport.shooterVelocityTarget);
        telemetry.addData("Flywheel Fire Tolerance", Transport.fireTolerance);
        telemetry.addData("Flywheel Ready", transport.inRange(Transport.shooterVelocity, Transport.shooterVelocityTarget));
        telemetry.addData("Voltage", voltageSensor.getVoltage());
        telemetry.addData("Voltage Multiplier", 12 / voltageSensor.getVoltage());
        telemetry.addData("Intake Power", Transport.intake.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Transfer Power", Transport.transfer.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }

    @Override
    public void stop() {}
}