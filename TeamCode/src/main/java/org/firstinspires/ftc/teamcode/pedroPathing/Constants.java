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

    // ---------------- DRIVE CONSTANTS ----------------
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.95)
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

    // ---------------- LOCALIZER CONSTANTS ----------------
    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.0019742365894729686)
            .strafeTicksToInches(0.0019917627182528697)
            .turnTicksToInches(0.002025566753320759)
            .leftPodY(8)
            .rightPodY(-8)
            .strafePodX(0)
            .rightEncoder_HardwareMapName("fakeMotor")
            .leftEncoder_HardwareMapName("frontRight")
            .strafeEncoder_HardwareMapName("intakeTop")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD);

    // ---------------- FOLLOWER CONSTANTS (Balanced) ----------------
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10)   // updated from 15 → 10 kg

            // Balanced zero-power accelerations
            .forwardZeroPowerAcceleration(-26)
            .lateralZeroPowerAcceleration(-34)

            // Balanced translational PIDF
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.12,   // P
                    0.0,    // I
                    0.015,  // D
                    0.015   // F
            ))

            // Balanced heading PIDF
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.55,   // P
                    0.0,    // I
                    0.05,   // D
                    0.015   // F
            ))

            // Drive PIDF (kept mild)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02,   // P
                    0.0,    // I
                    0.001,  // D
                    0.6,    // F
                    0.01    // filter
            ))

            .centripetalScaling(0.0045);

    // ---------------- PATH CONSTRAINTS (Balanced) ----------------
    public static PathConstraints pathConstraints =
            new PathConstraints(
                    0.90,   // maxVel
                    12,     // maxAccel
                    0.75,   // maxAngVel
                    0.75    // maxAngAccel
            );

    // ---------------- FOLLOWER BUILDER ----------------
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}