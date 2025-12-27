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
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous(name = "BlueGoal_SR", group = "Autonomous")
public class BlueGoal_SR extends LinearOpMode {

    private enum AutoState {
        DRIVE_PATH1A, //skipped B because you can combine into diagonal straight into C
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

    private ShooterSubsystem shooter;

    // ---------------- POSES ----------------
    private final Pose startPose     = new Pose(21, 122, Math.toRadians(-44));

    private final Pose midShootPose  = new Pose(52, 89.6, Math.toRadians(-34.7));
    private final Pose preShootPose  = new Pose(52, 80,   Math.toRadians(-34.7));
    private final Pose shootPose     = new Pose(55, 89.6, Math.toRadians(-34.7));

    private final Pose path2Pose  = new Pose(44, 84, Math.toRadians(180));
    private final Pose path3Pose  = new Pose(15, 84, Math.toRadians(180));
    private final Pose path5Pose  = new Pose(44, 60, Math.toRadians(180));
    private final Pose path6Pose  = new Pose(9, 60,  Math.toRadians(180));
    private final Pose path8Pose  = new Pose(44, 36, Math.toRadians(180));
    private final Pose path9Pose  = new Pose(9, 36,  Math.toRadians(180));
    private final Pose path11Pose = new Pose(35, 71, Math.toRadians(180));

    // ---------------- PATHS ----------------
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
        shooter = new ShooterSubsystem(hardwareMap);

        setDrivePower(0, 0, 0, 0);

        buildPaths();
        follower.setPose(startPose);

        currentState = AutoState.DRIVE_PATH1A;

        waitForStart();
        stateTimer.resetTimer();

        while (opModeIsActive() && currentState != AutoState.DONE) {

            follower.update();

            switch (currentState) {

                // ---------------- PATH 1A (diagonal start → midShoot) ----------------
                case DRIVE_PATH1A:
                    if (stateJustEntered()) {
                        shooter.setTargetRPM(RPMshot);
                        follower.followPath(path1A, false);
                    }

                    if (pathComplete(midShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH1C);
                    }
                    break;

                // ---------------- PATH 1C (settle-only) ----------------
                case DRIVE_PATH1C:
                    if (stateJustEntered()) {
                        try {
                            settleAtPose(shootPose, 1.0);
                        } catch (InterruptedException e) {}
                        transitionTo(AutoState.FIRE_BALLS);
                    }
                    break;

                // ---------------- SHOOT 1 ----------------
                case FIRE_BALLS:
                    if (stateJustEntered()) {
                        fireThreeBalls();
                        shooter.stopShooter();
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

                // ---------------- PATH 4C (settle-only) ----------------
                case DRIVE_PATH4C:
                    if (stateJustEntered()) {
                        try {
                            settleAtPose(shootPose, 1.0);
                        } catch (InterruptedException e) {}
                        transitionTo(AutoState.FIRE_BALLS1);
                    }
                    break;

                // ---------------- SHOOT 2 ----------------
                case FIRE_BALLS1:
                    if (stateJustEntered()) {
                        fireThreeBalls();
                        shooter.stopShooter();
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

                // ---------------- PATH 7C (settle-only) ----------------
                case DRIVE_PATH7C:
                    if (stateJustEntered()) {
                        try {
                            settleAtPose(shootPose, 1.0);
                        } catch (InterruptedException e) {}
                        transitionTo(AutoState.FIRE_BALLS2);
                    }
                    break;

                // ---------------- SHOOT 3 ----------------
                case FIRE_BALLS2:
                    if (stateJustEntered()) {
                        fireThreeBalls();
                        shooter.stopShooter();
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

                // ---------------- PATH 10C (settle-only) ----------------
                case DRIVE_PATH10C:
                    if (stateJustEntered()) {
                        try {
                            settleAtPose(shootPose, 1.0);
                        } catch (InterruptedException e) {}
                        transitionTo(AutoState.FIRE_BALLS3);
                    }
                    break;

                // ---------------- SHOOT 4 ----------------
                case FIRE_BALLS3:
                    if (stateJustEntered()) {
                        fireThreeBalls();
                        shooter.stopShooter();
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

    // ---------------- TRANSITION HELPERS ----------------
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

    // ---------------- SETTLE BEFORE SHOOTING ----------------
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

    // ---------------- SHOOTING ROUTINE ----------------
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

    // ---------------- BUILD PATHS ----------------
    private void buildPaths() {

        // Path 1A: start (-44) -> midShoot (-34.7) → LINEAR
        path1A = follower.pathBuilder()
                .addPath(new BezierLine(startPose, midShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), midShootPose.getHeading())
                .build();

        // Path 1C: midShoot (-34.7) -> shootPose (-34.7) → CONSTANT
        path1C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, shootPose))
                .setConstantHeadingInterpolation(midShootPose.getHeading())
                .build();

        // Path 2: shoot (-34.7) -> path2 (180) → LINEAR
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, path2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), path2Pose.getHeading())
                .build();

        // Path 3: path2 (180) -> path3 (180) → CONSTANT
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(path2Pose, path3Pose))
                .setConstantHeadingInterpolation(path2Pose.getHeading())
                .build();

        // Path 4A: path3 (180) -> preShoot (-34.7) → LINEAR
        path4A = follower.pathBuilder()
                .addPath(new BezierLine(path3Pose, preShootPose))
                .setLinearHeadingInterpolation(path3Pose.getHeading(), preShootPose.getHeading())
                .build();

        // Path 4B: preShoot (-34.7) -> midShoot (-34.7) → CONSTANT
        path4B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setConstantHeadingInterpolation(preShootPose.getHeading())
                .build();

        // Path 4C: midShoot (-34.7) -> shootPose (-34.7) → CONSTANT
        path4C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, shootPose))
                .setConstantHeadingInterpolation(midShootPose.getHeading())
                .build();

        // Path 5: shoot (-34.7) -> path5 (180) → LINEAR
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, path5Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), path5Pose.getHeading())
                .build();

        // Path 6: path5 (180) -> path6 (180) → CONSTANT
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(path5Pose, path6Pose))
                .setConstantHeadingInterpolation(path5Pose.getHeading())
                .build();

        // Path 7A: path6 (180) -> preShoot (-34.7) → LINEAR
        path7A = follower.pathBuilder()
                .addPath(new BezierLine(path6Pose, preShootPose))
                .setLinearHeadingInterpolation(path6Pose.getHeading(), preShootPose.getHeading())
                .build();

        // Path 7B: preShoot (-34.7) -> midShoot (-34.7) → CONSTANT
        path7B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setConstantHeadingInterpolation(preShootPose.getHeading())
                .build();

        // Path 7C: midShoot (-34.7) -> shootPose (-34.7) → CONSTANT
        path7C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, shootPose))
                .setConstantHeadingInterpolation(midShootPose.getHeading())
                .build();

        // Path 8: shoot (-34.7) -> path8 (180) → LINEAR
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, path8Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), path8Pose.getHeading())
                .build();

        // Path 9: path8 (180) -> path9 (180) → CONSTANT
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(path8Pose, path9Pose))
                .setConstantHeadingInterpolation(path8Pose.getHeading())
                .build();

        // Path 10A: path9 (180) -> preShoot (-34.7) → LINEAR
        path10A = follower.pathBuilder()
                .addPath(new BezierLine(path9Pose, preShootPose))
                .setLinearHeadingInterpolation(path9Pose.getHeading(), preShootPose.getHeading())
                .build();

        // Path 10B: preShoot (-34.7) -> midShoot (-34.7) → CONSTANT
        path10B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setConstantHeadingInterpolation(preShootPose.getHeading())
                .build();

        // Path 10C: midShoot (-34.7) -> shootPose (-34.7) → CONSTANT
        path10C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, shootPose))
                .setConstantHeadingInterpolation(midShootPose.getHeading())
                .build();

        // Path 11: shoot (-34.7) -> path11 (180) → LINEAR
        path11 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, path11Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), path11Pose.getHeading())
                .build();
    }
}