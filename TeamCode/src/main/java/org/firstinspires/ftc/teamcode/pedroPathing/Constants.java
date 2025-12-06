package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(.96)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(63.09182251326858)
            .yVelocity(51.26808593315976);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.0019742365894729686)
            .strafeTicksToInches(0.0019917627182528697)
            .turnTicksToInches(.002025566753320759)
            .leftPodY(8)
            .rightPodY(-8)
            .strafePodX(0)
            .rightEncoder_HardwareMapName("fakeMotor")
            .leftEncoder_HardwareMapName("frontRight")
            .strafeEncoder_HardwareMapName("intakeTop")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(15)
            .forwardZeroPowerAcceleration(-38.72960408654546)
            .lateralZeroPowerAcceleration(-55.23187564761812)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.02, 0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.1, 0.015))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0.0,0.001,0.6,0.01))
            .centripetalScaling(0.005);

    public static PathConstraints pathConstraints =
            new PathConstraints(0.92, 15, 0.80, 0.80);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
