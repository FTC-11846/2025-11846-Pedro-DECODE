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

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6)   //just a guess need to weigh this
            .forwardZeroPowerAcceleration(-44.95)
            .lateralZeroPowerAcceleration(-58.36)
            .centripetalScaling(0.00005)
            .headingPIDFCoefficients(new PIDFCoefficients(0,0.01,0,1.5))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.00001,0.01,0,0.03,0.6))
//            .translationalIntegral(new PIDFCoefficients(0,0.015,0,0.001))  <-- FollowerConstants doesn't have a setter method for this, so need to switch to tuning the standard translational, not the integral
            .translationalPIDFCoefficients(new PIDFCoefficients(0,0.15,0,0.001));   //  Not actually tuned yet, this is default values.
    // !!! Need to double-check if the values in Panels were in the same order as required by the Setter !!!    http://192.168.43.1:8001

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(42.48)
            .yVelocity(35.59);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3)
            .strafePodX(6)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static Follower createFollower(HardwareMap hardwareMap) {
        Follower build = new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
        return build;
    }
}
