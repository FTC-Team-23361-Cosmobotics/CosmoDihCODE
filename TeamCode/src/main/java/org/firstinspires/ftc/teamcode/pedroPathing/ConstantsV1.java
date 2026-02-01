package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
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

public class ConstantsV1 {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.882023308) //TODO: ADD MASS IN KG
            .forwardZeroPowerAcceleration(-34.60590778058413)
            .lateralZeroPowerAcceleration(-67.13642763656057)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(.07,0,.006,.05))
            .headingPIDFCoefficients(new PIDFCoefficients(.6, 0, .006, .05))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0,0.001,0.6,0.7))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(.15,0,.01,0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(.25,0,.001,0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0,0.001,0.6,0.1))
            .centripetalScaling(.0003)
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
            .xVelocity(89) //tuned value: 94.96201979269193
            .yVelocity(68) //tuned value: 73.79728686715674
            .nominalVoltage(13.75)
            .useVoltageCompensation(true)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.7795275591)
            .strafePodX(3.3070866142)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            ;
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 50, 1.25, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
