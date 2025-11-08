package org.firstinspires.ftc.teamcode.pedroPathing.auto;

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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.teleop.systems.Transport;
import org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars;

@Autonomous(name = "BlueGoalAuto: Nine Artifacts", group = "GoalAuto")
public class BlueGoalNineAuto extends OpMode {
    private VoltageSensor voltageSensor;
    private Follower follower;

    private Transport transport;
    private Timer pathTimer, opmodeTimer;
    private int pathState;


    /** POSE COORDINATES **/
    private final double spikeX = 12, highSpikeY = 80, midSpikeY = 55, lowSpikeY = 36;
    public static double startX = 20, startY = 122, startHeading = Math.toRadians(135);
    public static double scoreX = 43, scoreY = 107, scoreHeading = Math.toRadians(135);
    public static double parkX = 36, parkY = 72, parkHeading = Math.toRadians(180);

    /** START, SCORE, AND PARK POSES **/
    private final Pose startPose = new Pose(startX, startY, startHeading); // Start Pose of our robot.
    private final Pose scorePose = new Pose(scoreX, scoreY, scoreHeading); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose parkPose = new Pose(parkX, parkY, parkHeading);


    /** SPIKE CONTROL POINTS **/
    private final Pose highSpikeControlPtPose = new Pose(59, highSpikeY);
    private final Pose midSpikeControlPtPose = new Pose(80, midSpikeY);

    private final Pose lowSpikeControlPtHighPose = new Pose(68, 70);
    private final Pose lowSpikeControlPtLowPose = new Pose(72, lowSpikeY);

    /** SPIKE POSES **/
    private final Pose highSpikePose = new Pose(spikeX, highSpikeY, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose midSpikePose = new Pose(spikeX - 10, midSpikeY, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose lowSpikePose = new Pose(spikeX, lowSpikeY, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    /** PATH CHAINS **/
    private PathChain scorePreload, grabPPG, scorePPG, grabPGP, scorePGP, grabGPP, scoreGPP, leaveZone;
    public void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setConstantHeadingInterpolation(startHeading)
                .build();

        grabPPG = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, highSpikeControlPtPose, highSpikePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        scorePPG = follower.pathBuilder()
                .addPath(new BezierCurve(highSpikePose, highSpikeControlPtPose, scorePose))
                .setLinearHeadingInterpolation(highSpikePose.getHeading(), scorePose.getHeading() + Math.toRadians(7))
                .build();

        grabPGP = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, midSpikeControlPtPose, midSpikePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        scorePGP = follower.pathBuilder()
                .addPath(new BezierCurve(midSpikePose, midSpikeControlPtPose, new Pose(scorePose.getX() + 3.5, scorePose.getY() - 3.5, scorePose.getHeading() + Math.toRadians(7))))
                .setLinearHeadingInterpolation(midSpikePose.getHeading(), scorePose.getHeading() + Math.toRadians(7))
                .build();

        leaveZone = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, parkPose))
                .setLinearHeadingInterpolation(scoreHeading, parkHeading)
                .build();

        grabGPP = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, lowSpikeControlPtHighPose, lowSpikeControlPtLowPose, lowSpikePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        scoreGPP = follower.pathBuilder()
                .addPath(new BezierCurve(lowSpikePose, lowSpikeControlPtLowPose, lowSpikeControlPtHighPose, scorePose))
                .setTangentHeadingInterpolation()
                .build();
    }

    private final double shootPreloadWait = 2;

    private final double shootIntakedWait = 1.5;
    private final double intakeWait = .1;

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
                if (transport.inRange(Transport.shooterVelocity, Transport.shooterVelocityTarget)) {
                    transport.ricoTransport = Transport.RicoTransport.SHOOT;
                } else {
                    transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_SHORT;
                }
                if (pathTimer.getElapsedTimeSeconds() > (shootPreloadWait * 12 / voltageSensor.getVoltage())) {
                    transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_SHORT;
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
                    follower.followPath(scorePPG,true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;
            case 6:
                if (transport.inRange(Transport.shooterVelocity, Transport.shooterVelocityTarget)) {
                    transport.ricoTransport = Transport.RicoTransport.SHOOT;
                } else {
                    transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_SHORT;
                }
                if (pathTimer.getElapsedTimeSeconds() > (shootIntakedWait * 12 / voltageSensor.getVoltage())) {
                    transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_SHORT;
                    follower.followPath(grabPGP,true);
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
                    follower.followPath(scorePGP,true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;
            case 10:
                if ((pathTimer.getElapsedTimeSeconds() > .25) && transport.inRange(Transport.shooterVelocity, Transport.shooterVelocityTarget)) {
                    transport.ricoTransport = Transport.RicoTransport.SHOOT;
                } else {
                    transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_SHORT;
                }
                if (pathTimer.getElapsedTimeSeconds() > (shootIntakedWait * 12 / voltageSensor.getVoltage())) {
                    transport.ricoTransport = Transport.RicoTransport.POWER_SHOOTER_SHORT;
                    follower.followPath(leaveZone,true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
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
        GlobalVars.isRed = false;
        GlobalVars.inAuto = true;
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
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
        transport.update(12 / voltageSensor.getVoltage());
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
