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
            .mass(10.1)
            .forwardZeroPowerAcceleration(-30.545326912653497)
            .lateralZeroPowerAcceleration(-66.88707843511375)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0.01,0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.06, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.0001, 0.6,0.001));


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("backLeftMotor")
            .rightRearMotorName("frontLeftMotor")
            .leftRearMotorName("frontRightMotor")
            .leftFrontMotorName("backRightMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(85.5)
            .yVelocity(73.03549218365526);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-103.5/2.54)
            .strafePodX(42/2.54)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);



    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1.5,
                0.7);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
