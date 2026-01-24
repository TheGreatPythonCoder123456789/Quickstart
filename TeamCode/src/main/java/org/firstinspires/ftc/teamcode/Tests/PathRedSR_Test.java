package org.firstinspires.ftc.teamcode.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemCloseShooting;

@Autonomous(name = "PathRedSR_Test_OLD", group = "Autonomous")
public class PathRedSR_Test extends LinearOpMode {

    private enum AutoState {
        DRIVE_PATH1A,
        DRIVE_PATH1B,
        DRIVE_PATH1C,
        FIRE_BALLS,
        DRIVE_PATH2,
        DRIVE_PATH3,
        DRIVE_PATH4A,
        DRIVE_PATH4B,
        DRIVE_PATH4C,
        FIRE_BALLS1,
        DRIVE_PATH5,
        DRIVE_PATH6,
        DRIVE_PATH7A,
        DRIVE_PATH7B,
        DRIVE_PATH7C,
        FIRE_BALLS2,
        DRIVE_PATH8,
        DRIVE_PATH9,
        DRIVE_PATH10A,
        DRIVE_PATH10B,
        DRIVE_PATH10C,
        FIRE_BALLS3,
        DRIVE_PATH11,
        DONE
    }

    private Follower follower;
    private Timer stateTimer;

    private DcMotor intakeTop;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private ShooterSubsystemCloseShooting shooter;

    // ---------------- POSES ----------------

    //starting position
    private final Pose startPose = new Pose(88, 9, Math.toRadians(270));

    private final Pose preShootPose = new Pose(92, 80, Math.toRadians(-138));
    private final Pose midShootPose = new Pose(92, 89.6, Math.toRadians(-138));

    // NEW SR LAUNCHING POSE
    private final Pose launchingPose = new Pose(96.8, 91.8, Math.toRadians(-136.8));

    // Cycle poses
    private final Pose path2Pose = new Pose(96, 84, Math.toRadians(0)); //pick up ball pose
    private final Pose path3Pose = new Pose(129, 84, Math.toRadians(0)); //GRAB
    private final Pose path5Pose = new Pose(96, 60, Math.toRadians(0)); //pick up ball pose
    private final Pose path6Pose = new Pose(135, 60, Math.toRadians(0)); //GRAB
    private final Pose path8Pose = new Pose(96, 36, Math.toRadians(0)); //pick up ball pose
    private final Pose path9Pose = new Pose(135, 36, Math.toRadians(0)); //GRAB
    private final Pose path11Pose = new Pose(109, 71, Math.toRadians(0)); //get off of line

    // PATHS
    private PathChain path1A, path1B, path1C;
    private PathChain path2, path3;
    private PathChain path4A, path4B, path4C;
    private PathChain path5, path6;
    private PathChain path7A, path7B, path7C;
    private PathChain path8, path9;
    private PathChain path10A, path10B, path10C;
    private PathChain path11;

    private AutoState currentState;
    private AutoState lastState = null;

    private static final double POSE_TOLERANCE = 4.5;
    private static final double STATE_TIMEOUT = 3.0;
    private double RPMshot = 2040;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        stateTimer = new Timer();

        intakeTop = hardwareMap.get(DcMotor.class, "intakeTop");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        intakeTop.setPower(0);
        shooter = new ShooterSubsystemCloseShooting(hardwareMap);

        setDrivePower(0, 0, 0, 0);

        buildPaths();
        follower.setPose(startPose);

        currentState = AutoState.DRIVE_PATH1A;

        waitForStart();
        stateTimer.resetTimer();

        while (opModeIsActive() && currentState != AutoState.DONE) {

            follower.update();

            switch (currentState) {

                // ---------------- PATH 1A ----------------
                case DRIVE_PATH1A:
                    if (stateJustEntered()) {
                        shooter.setTargetRPM(RPMshot);
                        follower.followPath(path1A, false);
                    }
                    if (pathComplete(preShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH1B);
                    }
                    break;

                // ---------------- PATH 1B ----------------
                case DRIVE_PATH1B:
                    if (stateJustEntered()) {
                        follower.followPath(path1B, false);
                    }
                    if (pathComplete(midShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH1C);
                    }
                    break;

                // ---------------- PATH 1C ----------------
                case DRIVE_PATH1C:
                    if (stateJustEntered()) {
                        follower.followPath(path1C, false);
                    }
                    if (pathComplete(launchingPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.FIRE_BALLS);
                    }
                    break;

                // ---------------- SHOOTING ----------------
                case FIRE_BALLS:
                    if (stateJustEntered()) {
                        follower.breakFollowing();
                        setDrivePower(0,0,0,0);
                        shootAllBalls();
                        transitionTo(AutoState.DRIVE_PATH2);
                    }
                    break;
                // ---------------- PATH 2 ----------------
                case DRIVE_PATH2:
                    if (stateJustEntered()) {
                        follower.followPath(path2, false);
                        intakeTop.setPower(-1.0);
                    }
                    if (pathComplete(path2Pose) || timedOut(STATE_TIMEOUT)) {
                        intakeTop.setPower(0);
                        transitionTo(AutoState.DRIVE_PATH3);
                    }
                    break;

                // ---------------- PATH 3 ----------------
                case DRIVE_PATH3:
                    if (stateJustEntered()) {
                        follower.followPath(path3, false);
                        intakeTop.setPower(-1.0);
                    }
                    if (pathComplete(path3Pose) || timedOut(STATE_TIMEOUT)) {
                        intakeTop.setPower(0);
                        transitionTo(AutoState.DRIVE_PATH4A);
                    }
                    break;

                // ---------------- PATH 4A ----------------
                case DRIVE_PATH4A:
                    if (stateJustEntered()) {
                        shooter.setTargetRPM(RPMshot);
                        follower.followPath(path4A, false);
                    }
                    if (pathComplete(preShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH4B);
                    }
                    break;

                // ---------------- PATH 4B ----------------
                case DRIVE_PATH4B:
                    if (stateJustEntered()) {
                        follower.followPath(path4B, false);
                    }
                    if (pathComplete(midShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH4C);
                    }
                    break;

                // ---------------- PATH 4C ----------------
                case DRIVE_PATH4C:
                    if (stateJustEntered()) {
                        follower.followPath(path4C, false);
                    }
                    if (pathComplete(launchingPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.FIRE_BALLS1);
                    }
                    break;

                // ---------------- SHOOT 2 ----------------
                case FIRE_BALLS1:
                    if (stateJustEntered()) {
                        follower.breakFollowing();
                        setDrivePower(0,0,0,0);
                        shootAllBalls();
                        transitionTo(AutoState.DRIVE_PATH5);
                    }
                    break;

                // ---------------- PATH 5 ----------------
                case DRIVE_PATH5:
                    if (stateJustEntered()) {
                        follower.followPath(path5, false);
                        intakeTop.setPower(-1.0);
                    }
                    if (pathComplete(path5Pose) || timedOut(STATE_TIMEOUT)) {
                        intakeTop.setPower(0);
                        transitionTo(AutoState.DRIVE_PATH6);
                    }
                    break;

                // ---------------- PATH 6 ----------------
                case DRIVE_PATH6:
                    if (stateJustEntered()) {
                        follower.followPath(path6, false);
                        intakeTop.setPower(-1.0);
                    }
                    if (pathComplete(path6Pose) || timedOut(STATE_TIMEOUT)) {
                        intakeTop.setPower(0);
                        transitionTo(AutoState.DRIVE_PATH7A);
                    }
                    break;

                // ---------------- PATH 7A ----------------
                case DRIVE_PATH7A:
                    if (stateJustEntered()) {
                        shooter.setTargetRPM(RPMshot);
                        follower.followPath(path7A, false);
                    }
                    if (pathComplete(preShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH7B);
                    }
                    break;

                // ---------------- PATH 7B ----------------
                case DRIVE_PATH7B:
                    if (stateJustEntered()) {
                        follower.followPath(path7B, false);
                    }
                    if (pathComplete(midShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH7C);
                    }
                    break;

                // ---------------- PATH 7C ----------------
                case DRIVE_PATH7C:
                    if (stateJustEntered()) {
                        follower.followPath(path7C, false);
                    }
                    if (pathComplete(launchingPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.FIRE_BALLS2);
                    }
                    break;

                // ---------------- SHOOT 3 ----------------
                case FIRE_BALLS2:
                    if (stateJustEntered()) {
                        follower.breakFollowing();
                        setDrivePower(0,0,0,0);
                        shootAllBalls();
                        transitionTo(AutoState.DRIVE_PATH8);
                    }
                    break;

                // ---------------- PATH 8 ----------------
                case DRIVE_PATH8:
                    if (stateJustEntered()) {
                        follower.followPath(path8, false);
                        intakeTop.setPower(-1.0);
                    }
                    if (pathComplete(path8Pose) || timedOut(STATE_TIMEOUT)) {
                        intakeTop.setPower(0);
                        transitionTo(AutoState.DRIVE_PATH9);
                    }
                    break;

                // ---------------- PATH 9 ----------------
                case DRIVE_PATH9:
                    if (stateJustEntered()) {
                        follower.followPath(path9, false);
                        intakeTop.setPower(-1.0);
                    }
                    if (pathComplete(path9Pose) || timedOut(STATE_TIMEOUT)) {
                        intakeTop.setPower(0);
                        transitionTo(AutoState.DRIVE_PATH10A);
                    }
                    break;

                // ---------------- PATH 10A ----------------
                case DRIVE_PATH10A:
                    if (stateJustEntered()) {
                        shooter.setTargetRPM(RPMshot);
                        follower.followPath(path10A, false);
                    }
                    if (pathComplete(preShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH10B);
                    }
                    break;

                // ---------------- PATH 10B ----------------
                case DRIVE_PATH10B:
                    if (stateJustEntered()) {
                        follower.followPath(path10B, false);
                    }
                    if (pathComplete(midShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH10C);
                    }
                    break;

                // ---------------- PATH 10C ----------------
                case DRIVE_PATH10C:
                    if (stateJustEntered()) {
                        follower.followPath(path10C, false);
                    }
                    if (pathComplete(launchingPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.FIRE_BALLS3);
                    }
                    break;

                // ---------------- SHOOT 4 ----------------
                case FIRE_BALLS3:
                    if (stateJustEntered()) {
                        follower.breakFollowing();
                        setDrivePower(0,0,0,0);
                        shootAllBalls();
                        transitionTo(AutoState.DRIVE_PATH11);
                    }
                    break;

                // ---------------- PATH 11 ----------------
                case DRIVE_PATH11:
                    if (stateJustEntered()) {
                        follower.followPath(path11, false);
                    }
                    if (pathComplete(path11Pose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DONE);
                    }
                    break;

                case DONE:
                    break;
            }

            telemetry.addData("State", currentState);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }

        shooter.stopShooter();
        intakeTop.setPower(0);
        setDrivePower(0, 0, 0, 0);
    }

    // ---------------- STATE HELPERS ----------------

    private boolean stateJustEntered() {
        if (lastState != currentState) {
            lastState = currentState;
            stateTimer.resetTimer();
            return true;
        }
        return false;
    }

    private boolean timedOut(double timeoutSeconds) {
        return stateTimer.getElapsedTimeSeconds() > timeoutSeconds;
    }

    private void transitionTo(AutoState next) {
        currentState = next;
    }

    private boolean pathComplete(Pose target) {
        Pose current = follower.getPose();
        double dx = current.getX() - target.getX();
        double dy = current.getY() - target.getY();
        return Math.hypot(dx, dy) < POSE_TOLERANCE;
    }

    // ---------------- SHOOTING ROUTINE (SR) ----------------

    private void shootAllBalls() throws InterruptedException {

        shooter.setTargetRPM(2100);
        sleep(1500);

        // Ball 1
        intakeTop.setPower(-1.0);
        sleep(300);
        intakeTop.setPower(0.5);
        sleep(700);
        intakeTop.setPower(0);
        sleep(300);

        // Ball 2
        intakeTop.setPower(-1.0);
        sleep(500);
        intakeTop.setPower(0.5);
        sleep(700);
        intakeTop.setPower(0);
        sleep(300);

        // Ball 3
        intakeTop.setPower(-1.0);
        sleep(800);

        intakeTop.setPower(0);
        shooter.stopShooter();
    }

    // ------------ DRIVE POWER HELPER -------------
    private void setDrivePower(double lf, double rf, double lb, double rb) {
        frontLeft.setPower(lf);
        frontRight.setPower(rf);
        backLeft.setPower(lb);
        backRight.setPower(rb);
    }

    // ---------------- PATH CONSTRAINTS ----------------
    private static final PathConstraints FAST_CONSTRAINTS =
            new PathConstraints(60, 60, 360, 360);

    private static final PathConstraints SLOW_CONSTRAINTS =
            new PathConstraints(20, 20, 180, 180);

    // ---------------- BUILD PATHS ----------------
    private void buildPaths() {

        // Path 1A: start -> preShoot (fast)
        path1A = follower.pathBuilder()
                .addPath(new BezierLine(startPose, preShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), preShootPose.getHeading())
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        // Path 1B: preShoot -> midShoot (fast)
        path1B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setConstantHeadingInterpolation(preShootPose.getHeading())
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        // Path 1C: midShoot -> launchingPose (fast)
        path1C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, launchingPose))
                .setConstantHeadingInterpolation(midShootPose.getHeading())
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        // ---------------- SLOW PATHS BEGIN ----------------

        // Path 2: launchingPose -> path2 (SLOW)
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(launchingPose, path2Pose))
                .setLinearHeadingInterpolation(launchingPose.getHeading(), path2Pose.getHeading())
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        // Path 3: path2 -> path3 (SLOW)
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(path2Pose, path3Pose))
                .setConstantHeadingInterpolation(path2Pose.getHeading())
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        // Path 5: launchingPose -> path5 (SLOW)
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(launchingPose, path5Pose))
                .setLinearHeadingInterpolation(launchingPose.getHeading(), path5Pose.getHeading())
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        // Path 6: path5 -> path6 (SLOW)
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(path5Pose, path6Pose))
                .setConstantHeadingInterpolation(path5Pose.getHeading())
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        // Path 8: launchingPose -> path8 (SLOW)
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(launchingPose, path8Pose))
                .setLinearHeadingInterpolation(launchingPose.getHeading(), path8Pose.getHeading())
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        // Path 9: path8 -> path9 (SLOW)
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(path8Pose, path9Pose))
                .setConstantHeadingInterpolation(path8Pose.getHeading())
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        // ---------------- SLOW PATHS END ----------------

        // Path 4A: path3 -> preShoot (fast)
        path4A = follower.pathBuilder()
                .addPath(new BezierLine(path3Pose, preShootPose))
                .setLinearHeadingInterpolation(path3Pose.getHeading(), preShootPose.getHeading())
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        // Path 4B: preShoot -> midShoot (fast)
        path4B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setConstantHeadingInterpolation(preShootPose.getHeading())
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        // Path 4C: midShoot -> launchingPose (fast)
        path4C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, launchingPose))
                .setConstantHeadingInterpolation(midShootPose.getHeading())
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        // Path 7A: path6 -> preShoot (fast)
        path7A = follower.pathBuilder()
                .addPath(new BezierLine(path6Pose, preShootPose))
                .setLinearHeadingInterpolation(path6Pose.getHeading(), preShootPose.getHeading())
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        // Path 7B: preShoot -> midShoot (fast)
        path7B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setConstantHeadingInterpolation(preShootPose.getHeading())
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        // Path 7C: midShoot -> launchingPose (fast)
        path7C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, launchingPose))
                .setConstantHeadingInterpolation(midShootPose.getHeading())
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        // Path 10A: path9 -> preShoot (fast)
        path10A = follower.pathBuilder()
                .addPath(new BezierLine(path9Pose, preShootPose))
                .setLinearHeadingInterpolation(path9Pose.getHeading(), preShootPose.getHeading())
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        // Path 10B: preShoot -> midShoot (fast)
        path10B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setConstantHeadingInterpolation(preShootPose.getHeading())
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        // Path 10C: midShoot -> launchingPose (fast)
        path10C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, launchingPose))
                .setConstantHeadingInterpolation(midShootPose.getHeading())
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        // Path 11: launchingPose -> path11 (fast)
        path11 = follower.pathBuilder()
                .addPath(new BezierLine(launchingPose, path11Pose))
                .setLinearHeadingInterpolation(launchingPose.getHeading(), path11Pose.getHeading())
                .setConstraints(FAST_CONSTRAINTS)
                .build();
    }
}