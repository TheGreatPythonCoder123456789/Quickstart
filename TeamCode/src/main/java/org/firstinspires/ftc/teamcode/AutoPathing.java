package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class AutoPathing extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // Motor declarations
    private DcMotor shootLeft, shootRight, intakeTop;

    // Fixed power settings
    private static final double INTAKE_POWER = 0.8;
    private static final double INTAKE_REVERSE_POWER = -0.8;

    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_INTAKE_POS,
        INTAKE_RING,
        DRIVE_INTAKE_ENDPOS
    }

    PathState pathState;

    // Updated poses with your coordinates
    private final Pose startPose = new Pose(87, 9, Math.toRadians(270));
    private final Pose shootPose = new Pose(84, 99, Math.toRadians(-143));
    private final Pose intakePose = new Pose(100, 84, Math.toRadians(0));
    private final Pose endPose = new Pose(100, 84, Math.toRadians(0)); // Same as intake position

    private PathChain driveStartPosShootPos, driveShootPosIntakePos, driveIntakeEndPos;

    public void buildPaths() {
        // Path 1: Start position to shoot position
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // Path 2: Shoot position to intake position
        driveShootPosIntakePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intakePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakePose.getHeading())
                .build();

        // Path 3: Intake position to end position (minimal movement)
        driveIntakeEndPos = follower.pathBuilder()
                .addPath(new BezierLine(intakePose, endPose))
                .setLinearHeadingInterpolation(intakePose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                if (follower.isBusy()) {
                    // Still following the path
                } else {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                double shootTime = pathTimer.getElapsedTimeSeconds();

                // Always run shooters during shooting sequence
                setShooterPower(0); // SET SHOOTER POWER DURING COMPETITION

                if (shootTime >= 6.0) {
                    // After 6 seconds, stop everything and move to next path
                    setShooterPower(0);
                    intakeTop.setPower(0);
                    follower.followPath(driveShootPosIntakePos, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_INTAKE_POS);
                } else {
                    // Sequence for shooting 3 balls:
                    if (shootTime < 3.0) {
                        // First 3 seconds: Warm up shooters only (no intake)
                        intakeTop.setPower(0);
                    } else if (shootTime < 4.0) {
                        // Second 1 second: Shoot first and second ball
                        intakeTop.setPower(INTAKE_REVERSE_POWER);
                    } else if (shootTime < 5.0) {
                        // Third 1 second: Shoot second ball
                        intakeTop.setPower(0);
                    } else {
                        // Fourth 1 second: Shoot third ball
                        intakeTop.setPower(INTAKE_REVERSE_POWER);
                    }
                }
                break;

            case DRIVE_SHOOTPOS_INTAKE_POS:
                // Wait for path completion or 10 second timeout
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 10) {
                    // Start intake when we reach intake position
                    intakeTop.setPower(INTAKE_POWER);
                    setPathState(PathState.INTAKE_RING);
                }
                break;

            case INTAKE_RING:
                // Run intake for 2 seconds
                if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                    intakeTop.setPower(0); // Stop intake after 2 seconds
                    follower.followPath(driveIntakeEndPos, true);
                    setPathState(PathState.DRIVE_INTAKE_ENDPOS);
                }
                break;

            case DRIVE_INTAKE_ENDPOS:
                // Wait for path completion or 10 second timeout
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 10) {
                    telemetry.addLine("Done all Paths");
                }
                break;

            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    // Helper method to set both shooter powers at once
    private void setShooterPower(double power) {
        shootLeft.setPower(power);
        shootRight.setPower(power);
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        // Initialize follower
        follower = Constants.createFollower(hardwareMap);

        // Initialize shooter and intake motors
        shootLeft = hardwareMap.get(DcMotor.class, "shootLeft");
        shootRight = hardwareMap.get(DcMotor.class, "shootRight");
        intakeTop = hardwareMap.get(DcMotor.class, "intakeTop");

        // Configure motor directions (adjust if needed)
        shootLeft.setDirection(DcMotor.Direction.FORWARD);
        shootRight.setDirection(DcMotor.Direction.FORWARD);
        intakeTop.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to zero power initially
        setShooterPower(0);
        intakeTop.setPower(0);

        buildPaths();
        follower.setPose(startPose);
        follower.followPath(driveStartPosShootPos, true);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Shooter Left Power", shootLeft.getPower());
        telemetry.addData("Shooter Right Power", shootRight.getPower());
        telemetry.addData("Intake Power", intakeTop.getPower());
    }
}