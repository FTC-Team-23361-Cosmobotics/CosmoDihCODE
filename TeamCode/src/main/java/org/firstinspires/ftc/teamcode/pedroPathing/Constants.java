package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.336) //TODO: ADD MASS IN KG
            .forwardZeroPowerAcceleration(-29.06798)
            .lateralZeroPowerAcceleration(-60.3506)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .translationalPIDFCoefficients(new PIDFCoefficients(.1,0,0,0))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0, .01))
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(71.03569)
            .yVelocity(48.69649)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.30708661) //TODO: FIND ODO OFFSETS FOR YPOD (IN INCHES)
            .strafePodX(6.6147323) //TODO: FIND ODO OFFSETS FOR XPOD (IN INCHES)
            //TODO: TEST IF ODO IS IN CORRECT DIR USING LOCALIZATION TEST, IF NOT SWITCH ODO DIR BELOW
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint") //TODO: RENAME TO PINPOINT HARDWARE MAP NAME
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            ;
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
