package org.firstinspires.ftc.teamcode.pedroPathing;

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
            .forwardTicksToInches(0.0019742365894729686) // check and redo (direction for encoders)
            .strafeTicksToInches(0.0019917627182528697) // check and redo (direction for encoders)
            .turnTicksToInches(.002025566753320759) //check and redo (direction for encoders)
            .leftPodY(8)// https://pedropathing.com/docs/pathing/tuning/localization/three-wheel
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
            .lateralZeroPowerAcceleration(-55.23187564761812);
            //.translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0));
//hello
    public static PathConstraints pathConstraints = new PathConstraints(0.98, 15, 0.7, 0.9);
    /*
            tValueConstraint,    // When to consider path "complete" (0.0-1.0)
            timeoutConstraint,   // Maximum time allowed for path (seconds)
            brakingStrength,     // How aggressively to brake at end (0.0-1.0)
            brakingStart         // When to start braking (0.0-1.0 of path completion)
            0.98,   // tValueConstraint: Stop when 98% of path is complete
            15.0,    // timeoutConstraint: 15 second maximum per path
            0.7,    // brakingStrength: Moderate braking (70% strength)
            0.9     // brakingStart: Start braking at 90% of path completion
    */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
