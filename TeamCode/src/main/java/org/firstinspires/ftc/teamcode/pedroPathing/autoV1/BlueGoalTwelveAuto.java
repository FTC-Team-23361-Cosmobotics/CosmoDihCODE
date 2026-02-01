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

@Autonomous(name = "BlueGoalAuto: Twelve Artifacts", group = "GoalAuto", preselectTeleOp = "RedDihCodeTeleop")
public class BlueGoalTwelveAuto extends OpMode {
    private VoltageSensor voltageSensor;
    private Follower follower;

    private Transport transport;
    private Timer pathTimer, opmodeTimer;
    private int pathState;


    /** POSE COORDINATES **/
    public static double startX = 17.7924217463, startY = 119.32784184514004, startHeading = Math.toRadians(146);
    public static double spikeX = 16, highSpikeY = 84, midSpikeY = 60, lowSpikeY = 36;
    public static double scoreX = 42, scoreY = 102, scoreMedX = 60, scoreMedY = 84, scoreHeading = Math.toRadians(135);

    public static double parkX = 64, parkY = 98, parkHeading = Math.toRadians(135);
    public static double gateX = 15, gateY = 70, gateHeading = Math.toRadians(0);

    /** START, SCORE, GATE, AND PARK POSES **/
    private final Pose startPose = new Pose(startX, startY, startHeading); // Start Pose of our robot.
    private final Pose scorePose = new Pose(scoreX, scoreY, scoreHeading); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose mediumScorePose = new Pose(scoreMedX, scoreMedY, scoreHeading);
    private final Pose parkPose = new Pose(parkX, parkY, parkHeading);
    private final Pose gatePose = new Pose(gateX, gateY, gateHeading);


    /** CONTROL POINTS **/
    private final Pose gateControlPtPose = new Pose(40, 77);
    private final Pose highSpikeControlPtPose = new Pose(59, highSpikeY);
    private final Pose midSpikeControlPtPose = new Pose(59, midSpikeY);

    private final Pose lowSpikeControlPtHighPose = new Pose(59, 70);
    private final Pose lowSpikeControlPtLowPose = new Pose(59, lowSpikeY);

    /** SPIKE POSES **/
    private final Pose highSpikePose = new Pose(spikeX, highSpikeY, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose midSpikePose = new Pose(spikeX + 7, midSpikeY, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose lowSpikePose = new Pose(spikeX + 7, lowSpikeY, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    /** PATH CHAINS **/

    private PathChain scorePreload, grabPPG, tapGate, scorePPG, grabPGP, scorePGP, grabGPP, scoreGPP, leaveZone;
    public void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startHeading, scoreHeading)
                .build();

        grabPPG = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, highSpikeControlPtPose, highSpikePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        tapGate = follower.pathBuilder()
                .addPath(new BezierCurve(highSpikePose, gateControlPtPose, gatePose))
                .setConstantHeadingInterpolation(gateHeading)
                .build();

        scorePPG = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, highSpikeControlPtPose, scorePose))
                .setLinearHeadingInterpolation(gateHeading, scorePose.getHeading())
                .build();

        grabPGP = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, midSpikeControlPtPose, midSpikePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        scorePGP = follower.pathBuilder()
                .addPath(new BezierCurve(midSpikePose, midSpikeControlPtPose, scorePose))
                .setLinearHeadingInterpolation(midSpikePose.getHeading(), scoreHeading)
                .build();

        grabGPP = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, lowSpikeControlPtHighPose, lowSpikeControlPtLowPose, lowSpikePose))
                .setLinearHeadingInterpolation(scoreHeading, lowSpikePose.getHeading())
                .build();

        scoreGPP = follower.pathBuilder()
                .addPath(new BezierLine(lowSpikePose, scorePose))
                .setLinearHeadingInterpolation(lowSpikePose.getHeading(), scoreHeading)
                .build();

        leaveZone = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scoreHeading, parkHeading)
                .build();
    }

    private final double shootBuffer = .35;
    private final double shootWait = shootBuffer + 1.5;
    private final double intakeWait = .25;
    private final double tapWait = .75;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_SHORT;
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > shootBuffer && transport.inRange(Transport.shooterVelocity, Transport.shooterVelocityTarget)) {
                    transport.ricoTransport = Transport.RicoTransport.SHOOT;
                } else {
                    transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_SHORT;
                }
                if (pathTimer.getElapsedTimeSeconds() > shootWait) {
                    transport.ricoTransport = Transport.RicoTransport.INTAKE;
                    follower.followPath(grabPPG,true);
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
                    follower.followPath(tapGate,true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds() > tapWait) {
                    transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_SHORT;
                    follower.followPath(scorePPG,true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > shootBuffer && transport.inRange(Transport.shooterVelocity, Transport.shooterVelocityTarget)) {
                    transport.ricoTransport = Transport.RicoTransport.SHOOT;
                } else {
                    transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_SHORT;
                }
                if (pathTimer.getElapsedTimeSeconds() > shootWait) {
                    transport.ricoTransport = Transport.RicoTransport.INTAKE;
                    follower.followPath(grabPGP,true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTimeSeconds() > intakeWait) {
                    transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_SHORT;
                    follower.followPath(scorePGP,true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() > shootBuffer && transport.inRange(Transport.shooterVelocity, Transport.shooterVelocityTarget)) {
                    transport.ricoTransport = Transport.RicoTransport.SHOOT;
                } else {
                    transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_SHORT;
                }
                if (pathTimer.getElapsedTimeSeconds() > shootWait) {
                    transport.ricoTransport = Transport.RicoTransport.INTAKE;
                    follower.followPath(grabGPP,true);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds() > intakeWait) {
                    transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_SHORT;
                    follower.followPath(scoreGPP,true);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTimeSeconds() > shootBuffer && transport.inRange(Transport.shooterVelocity, Transport.shooterVelocityTarget)) {
                    transport.ricoTransport = Transport.RicoTransport.SHOOT;
                } else {
                    transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_SHORT;
                }
                if (pathTimer.getElapsedTimeSeconds() > shootWait) {
                    transport.ricoTransport = Transport.RicoTransport.HOME;
                    follower.followPath(leaveZone,true);
                    setPathState(17);
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    transitionHeading = follower.getHeading();
                    setPathState(-1);
                }
                break;
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