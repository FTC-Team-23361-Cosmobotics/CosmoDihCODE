package org.firstinspires.ftc.teamcode.pedroPathing.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.teleop.systems.Transport;
import org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars;

@Autonomous(name = "RedGoalAuto", group = "Auto")
public class RedGoalAuto extends OpMode {
    private Follower follower;

    private Transport transport;
    private Timer pathTimer, opmodeTimer;
    private int pathState;


    /** SPIKE COORDINATES **/
    private final double spikeX = 125, highSpikeY = 84, midSpikeY = 60, lowSpikeY = 36;
    private final double startHeading = Math.toRadians(45);

    /** START AND SCORE POSES **/
    private final Pose startPose = new Pose(124, 122); // Start Pose of our robot.
    private final Pose scorePose = new Pose(102, 102); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


    /** SPIKE CONTROL POINTS **/
    private final Pose highSpikeControlPtPose = new Pose(85, highSpikeY);
    private final Pose midSpikeControlPtPose = new Pose(64, midSpikeY);

    private final Pose lowSpikeControlPtHighPose = new Pose(76, 70);
    private final Pose lowSpikeControlPtLowPose = new Pose(72, lowSpikeY);

    /** SPIKE POSES **/
    private final Pose highSpikePose = new Pose(spikeX, highSpikeY); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose midSpikePose = new Pose(spikeX, midSpikeY); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose lowSpikePose = new Pose(spikeX, lowSpikeY); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private PathChain scorePreload, grabPPG, scorePPG, grabPGP, scorePGP, grabGPP, scoreGPP;
    public void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setConstantHeadingInterpolation(startHeading)
                .build();

        grabPPG = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, highSpikeControlPtPose, highSpikePose))
                .setTangentHeadingInterpolation()
                .build();

        scorePPG = follower.pathBuilder()
                .addPath(new BezierCurve(highSpikePose, highSpikeControlPtPose, scorePose))
                .setTangentHeadingInterpolation()
                .build();

        grabPGP = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, midSpikeControlPtPose, midSpikePose))
                .setTangentHeadingInterpolation()
                .build();

        scorePGP = follower.pathBuilder()
                .addPath(new BezierCurve(midSpikePose, midSpikeControlPtPose, scorePose))
                .setTangentHeadingInterpolation()
                .build();

        grabGPP = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, lowSpikeControlPtHighPose, lowSpikeControlPtLowPose, lowSpikePose))
                .setTangentHeadingInterpolation()
                .build();

        scoreGPP = follower.pathBuilder()
                .addPath(new BezierCurve(lowSpikePose, lowSpikeControlPtLowPose, lowSpikeControlPtHighPose, scorePose))
                .setTangentHeadingInterpolation()
                .build();
    }

    /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
    */

    private final double shootWait = 1.5;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_SHORT;
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    transport.ricoTransport = Transport.RicoTransport.SHOOT;
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > shootWait) {
                    transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_SHORT;
                    follower.followPath(grabPPG,true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(scorePPG,true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    transport.ricoTransport = Transport.RicoTransport.SHOOT;
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds() > shootWait) {
                    transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_SHORT;
                    follower.followPath(grabPGP,true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePGP,true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    transport.ricoTransport = Transport.RicoTransport.SHOOT;
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > shootWait) {
                    transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_SHORT;
                    follower.followPath(grabGPP,true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(scoreGPP, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    transport.ricoTransport = Transport.RicoTransport.SHOOT;
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > shootWait) {
                    transport.ricoTransport = Transport.RicoTransport.HOME;
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
        transport = new Transport(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
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
        transport.update();
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {}
}