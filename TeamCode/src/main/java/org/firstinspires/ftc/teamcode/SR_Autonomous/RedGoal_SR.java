package org.firstinspires.ftc.teamcode.SR_Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemCloseShooting;

@Autonomous(name = "RedGoal_SR", group = "Autonomous")
public class RedGoal_SR extends LinearOpMode {

    private enum AutoState {
        DRIVE_PATH1A,
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

    // -------- POSES (MIRRORED FROM BLUEGOAL) --------
    // Blue start: (21, 122, -44°) → Red: (123, 122, -136°)
    private final Pose startPose     = new Pose(123, 122, Math.toRadians(-136));

    // Blue midShoot: (52, 89.6, -34.7°) → Red: (92, 89.6, -145.3°)
    private final Pose midShootPose  = new Pose(92, 89.6, Math.toRadians(-145.3));
    // Blue preShoot: (52, 80, -34.7°) → Red: (92, 80, -145.3°)
    private final Pose preShootPose  = new Pose(92, 80,   Math.toRadians(-145.3));
    // Blue shoot: (55, 89.6, -34.7°) → Red: (89, 89.6, -145.3°)
    private final Pose shootPose     = new Pose(89, 89.6, Math.toRadians(-145.3));

    // Blue path2: (44, 84, 180°) → Red: (100, 84, 0°)
    private final Pose path2Pose  = new Pose(100, 84, Math.toRadians(0));
    // Blue path3: (15, 84, 180°) → Red: (129, 84, 0°)
    private final Pose path3Pose  = new Pose(129, 84, Math.toRadians(0));
    // Blue path5: (44, 60, 180°) → Red: (100, 60, 0°)
    private final Pose path5Pose  = new Pose(100, 60, Math.toRadians(0));
    // Blue path6: (9, 60, 180°) → Red: (135, 60, 0°)
    private final Pose path6Pose  = new Pose(135, 60, Math.toRadians(0));
    // Blue path8: (44, 36, 180°) → Red: (100, 36, 0°)
    private final Pose path8Pose  = new Pose(100, 36, Math.toRadians(0));
    // Blue path9: (9, 36, 180°) → Red: (135, 36, 0°)
    private final Pose path9Pose  = new Pose(135, 36, Math.toRadians(0));
    // Blue path11: (35, 71, 180°) → Red: (109, 71, 0°)
    private final Pose path11Pose = new Pose(109, 71, Math.toRadians(0));

    // -------- PATHS --------
    private PathChain path1A, path1C;
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
    private static final double STATE_TIMEOUT  = 3.0;
    private double RPMshot = 2040;

    @Override
    public void runOpMode() throws InterruptedException {

        follower   = Constants.createFollower(hardwareMap);
        stateTimer = new Timer();

        intakeTop  = hardwareMap.get(DcMotor.class, "intakeTop");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

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

                case DRIVE_PATH1A:
                    if (stateJustEntered()) {
                        shooter.setTargetRPM(RPMshot);
                        follower.followPath(path1A, false);
                    }
                    if (pathComplete(midShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH1C);
                    }
                    break;

                case DRIVE_PATH1C:
                    if (stateJustEntered()) {
                        try { settleAtPose(shootPose, 1.0); } catch (InterruptedException e) {}
                        transitionTo(AutoState.FIRE_BALLS);
                    }
                    break;

                case FIRE_BALLS:
                    if (stateJustEntered()) {
                        fireThreeBalls();
                        shooter.stopShooter();
                        transitionTo(AutoState.DRIVE_PATH2);
                    }
                    break;

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

                case DRIVE_PATH4A:
                    if (stateJustEntered()) {
                        shooter.setTargetRPM(RPMshot);
                        follower.followPath(path4A, false);
                    }
                    if (pathComplete(preShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH4B);
                    }
                    break;

                case DRIVE_PATH4B:
                    if (stateJustEntered()) {
                        follower.followPath(path4B, false);
                    }
                    if (pathComplete(midShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH4C);
                    }
                    break;

                case DRIVE_PATH4C:
                    if (stateJustEntered()) {
                        try { settleAtPose(shootPose, 1.0); } catch (InterruptedException e) {}
                        transitionTo(AutoState.FIRE_BALLS1);
                    }
                    break;

                case FIRE_BALLS1:
                    if (stateJustEntered()) {
                        fireThreeBalls();
                        shooter.stopShooter();
                        transitionTo(AutoState.DRIVE_PATH5);
                    }
                    break;

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

                case DRIVE_PATH7A:
                    if (stateJustEntered()) {
                        shooter.setTargetRPM(RPMshot);
                        follower.followPath(path7A, false);
                    }
                    if (pathComplete(preShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH7B);
                    }
                    break;

                case DRIVE_PATH7B:
                    if (stateJustEntered()) {
                        follower.followPath(path7B, false);
                    }
                    if (pathComplete(midShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH7C);
                    }
                    break;

                case DRIVE_PATH7C:
                    if (stateJustEntered()) {
                        try { settleAtPose(shootPose, 1.0); } catch (InterruptedException e) {}
                        transitionTo(AutoState.FIRE_BALLS2);
                    }
                    break;

                case FIRE_BALLS2:
                    if (stateJustEntered()) {
                        fireThreeBalls();
                        shooter.stopShooter();
                        transitionTo(AutoState.DRIVE_PATH8);
                    }
                    break;

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

                case DRIVE_PATH10A:
                    if (stateJustEntered()) {
                        shooter.setTargetRPM(RPMshot);
                        follower.followPath(path10A, false);
                    }
                    if (pathComplete(preShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH10B);
                    }
                    break;

                case DRIVE_PATH10B:
                    if (stateJustEntered()) {
                        follower.followPath(path10B, false);
                    }
                    if (pathComplete(midShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH10C);
                    }
                    break;

                case DRIVE_PATH10C:
                    if (stateJustEntered()) {
                        try { settleAtPose(shootPose, 1.0); } catch (InterruptedException e) {}
                        transitionTo(AutoState.FIRE_BALLS3);
                    }
                    break;

                case FIRE_BALLS3:
                    if (stateJustEntered()) {
                        fireThreeBalls();
                        shooter.stopShooter();
                        transitionTo(AutoState.DRIVE_PATH11);
                    }
                    break;

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

    // -------- STATE HELPERS --------
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

    // -------- TRANSITION HELPERS --------
    private boolean pathComplete(Pose target) {
        Pose current = follower.getPose();
        double dx = current.getX() - target.getX();
        double dy = current.getY() - target.getY();
        return Math.hypot(dx, dy) < POSE_TOLERANCE;
    }

    private boolean atTargetPose(Pose target) {
        Pose current = follower.getPose();
        double dx = current.getX() - target.getX();
        double dy = current.getY() - target.getY();
        return Math.hypot(dx, dy) < POSE_TOLERANCE;
    }

    private boolean headingAligned(Pose target) {
        double error = Math.abs(
                Math.toDegrees(follower.getPose().getHeading() - target.getHeading())
        );
        return error < 3.0;
    }

    // -------- SETTLE BEFORE SHOOTING --------
    private void settleAtPose(Pose target) throws InterruptedException {
        settleAtPose(target, 0.4);
    }

    private void settleAtPose(Pose target, double settleSeconds) throws InterruptedException {
        setDrivePower(0, 0, 0, 0);

        sleep(300);

        long start = System.currentTimeMillis();
        long duration = (long)(settleSeconds * 1000);

        while (opModeIsActive() && System.currentTimeMillis() - start < duration) {
            follower.update();
            sleep(10);
        }

        sleep(150);
    }

    // -------- SHOOTING ROUTINE --------
    private void fireThreeBalls() throws InterruptedException {
        intakeTop.setPower(-1.0);
        sleep(200);
        intakeTop.setPower(0);
        sleep(500);

        intakeTop.setPower(-1.0);
        sleep(400);
        intakeTop.setPower(0);
        sleep(500);

        intakeTop.setPower(-1.0);
        sleep(600);
        intakeTop.setPower(0);
        sleep(300);
    }

    private void setDrivePower(double lf, double rf, double lb, double rb) {
        frontLeft.setPower(lf);
        frontRight.setPower(rf);
        backLeft.setPower(lb);
        backRight.setPower(rb);
    }

    // -------- BUILD PATHS --------
    private void buildPaths() {

        // Path 1A: start (-136) -> midShoot (-145.3) → LINEAR
        path1A = follower.pathBuilder()
                .addPath(new BezierLine(startPose, midShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), midShootPose.getHeading())
                .build();

        // Path 1C: midShoot (-145.3) -> shootPose (-145.3) → CONSTANT
        path1C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, shootPose))
                .setConstantHeadingInterpolation(midShootPose.getHeading())
                .build();

        // Path 2: shoot (-145.3) -> path2 (0) → LINEAR
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, path2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), path2Pose.getHeading())
                .build();

        // Path 3: path2 (0) -> path3 (0) → CONSTANT
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(path2Pose, path3Pose))
                .setConstantHeadingInterpolation(path2Pose.getHeading())
                .build();

        // Path 4A: path3 (0) -> preShoot (-145.3) → LINEAR
        path4A = follower.pathBuilder()
                .addPath(new BezierLine(path3Pose, preShootPose))
                .setLinearHeadingInterpolation(path3Pose.getHeading(), preShootPose.getHeading())
                .build();

        // Path 4B: preShoot (-145.3) -> midShoot (-145.3) → CONSTANT
        path4B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setConstantHeadingInterpolation(preShootPose.getHeading())
                .build();

        // Path 4C: midShoot (-145.3) -> shootPose (-145.3) → CONSTANT
        path4C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, shootPose))
                .setConstantHeadingInterpolation(midShootPose.getHeading())
                .build();

        // Path 5: shoot (-145.3) -> path5 (0) → LINEAR
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, path5Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), path5Pose.getHeading())
                .build();

        // Path 6: path5 (0) -> path6 (0) → CONSTANT
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(path5Pose, path6Pose))
                .setConstantHeadingInterpolation(path5Pose.getHeading())
                .build();

        // Path 7A: path6 (0) -> preShoot (-145.3) → LINEAR
        path7A = follower.pathBuilder()
                .addPath(new BezierLine(path6Pose, preShootPose))
                .setLinearHeadingInterpolation(path6Pose.getHeading(), preShootPose.getHeading())
                .build();

        // Path 7B: preShoot (-145.3) -> midShoot (-145.3) → CONSTANT
        path7B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setConstantHeadingInterpolation(preShootPose.getHeading())
                .build();

        // Path 7C: midShoot (-145.3) -> shootPose (-145.3) → CONSTANT
        path7C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, shootPose))
                .setConstantHeadingInterpolation(midShootPose.getHeading())
                .build();

        // Path 8: shoot (-145.3) -> path8 (0) → LINEAR
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, path8Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), path8Pose.getHeading())
                .build();

        // Path 9: path8 (0) -> path9 (0) → CONSTANT
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(path8Pose, path9Pose))
                .setConstantHeadingInterpolation(path8Pose.getHeading())
                .build();

        // Path 10A: path9 (0) -> preShoot (-145.3) → LINEAR
        path10A = follower.pathBuilder()
                .addPath(new BezierLine(path9Pose, preShootPose))
                .setLinearHeadingInterpolation(path9Pose.getHeading(), preShootPose.getHeading())
                .build();

        // Path 10B: preShoot (-145.3) -> midShoot (-145.3) → CONSTANT
        path10B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setConstantHeadingInterpolation(preShootPose.getHeading())
                .build();

        // Path 10C: midShoot (-145.3) -> shootPose (-145.3) → CONSTANT
        path10C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, shootPose))
                .setConstantHeadingInterpolation(midShootPose.getHeading())
                .build();

        // Path 11: shoot (-145.3) -> path11 (0) → LINEAR
        path11 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, path11Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), path11Pose.getHeading())
                .build();
    }
}