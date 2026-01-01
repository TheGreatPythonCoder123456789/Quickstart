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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous(name = "PathRedSR_Test_NEW", group = "Autonomous")
public class PathRedSR_Test_NEW extends LinearOpMode {

    private enum AutoState {
        DRIVE_PATH1A, DRIVE_PATH1B, DRIVE_PATH1C,
        FIRE_BALLS,
        DRIVE_PATH2_APPROACH, DRIVE_PATH2, DRIVE_PATH3,
        DRIVE_PATH4A, DRIVE_PATH4B, DRIVE_PATH4C,
        FIRE_BALLS1,
        DRIVE_PATH5_APPROACH, DRIVE_PATH5, DRIVE_PATH6,
        DRIVE_PATH7A, DRIVE_PATH7B, DRIVE_PATH7C,
        FIRE_BALLS2,
        DRIVE_PATH8_APPROACH, DRIVE_PATH8, DRIVE_PATH9,
        DRIVE_PATH10A, DRIVE_PATH10B, DRIVE_PATH10C,
        FIRE_BALLS3,
        DRIVE_PATH11,
        DONE
    }

    private Follower follower;
    private Timer stateTimer;

    private DcMotor intakeTop;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private ShooterSubsystem shooter;

    // NEW: gate servo
    private Servo gate;

    // Your backNum value for servoSetter
    private double backNum = 80;  // you can change to 90 if needed

    // ---------------- POSES ----------------
    private final Pose startPose = new Pose(88, 9, Math.toRadians(270));
    private final Pose preShootPose = new Pose(92.0, 80.0, Math.toRadians(-136.8));
    private final Pose midShootPose = new Pose(92.0, 89.6, Math.toRadians(-136.8));
    private final Pose launchingPose = new Pose(93.0, 96.0, Math.toRadians(-136.8));

    private final Pose row1ApproachPose = new Pose(92, 92, Math.toRadians(0));
    private final Pose row2ApproachPose = new Pose(92, 68, Math.toRadians(0));
    private final Pose row3ApproachPose = new Pose(92, 44, Math.toRadians(0));

    private final Pose path2Pose = new Pose(92, 84, Math.toRadians(0));
    private final Pose path3Pose = new Pose(129, 84, Math.toRadians(0));
    private final Pose path5Pose = new Pose(92, 60, Math.toRadians(0));
    private final Pose path6Pose = new Pose(135, 60, Math.toRadians(0));
    private final Pose path8Pose = new Pose(92, 36, Math.toRadians(0));
    private final Pose path9Pose = new Pose(135, 36, Math.toRadians(0));
    private final Pose path11Pose = new Pose(109, 71, Math.toRadians(0));

    // PATHS
    private PathChain path1A, path1B, path1C;
    private PathChain path2Approach, path2, path3;
    private PathChain path5Approach, path5, path6;
    private PathChain path8Approach, path8, path9;
    private PathChain path4A, path4B, path4C;
    private PathChain path7A, path7B, path7C;
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

        // NEW: gate servo
        gate = hardwareMap.get(Servo.class, "gate");

        // Force gate closed at start
        gate.setPosition(1.0);

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
                        setDrivePower(0, 0, 0, 0);
                        shootAllBalls();
                        transitionTo(AutoState.DRIVE_PATH2_APPROACH);
                    }
                    break;

                // ---------------- PATH 2 APPROACH ----------------
                case DRIVE_PATH2_APPROACH:
                    if (stateJustEntered()) {
                        follower.followPath(path2Approach, false);
                    }
                    if (pathComplete(row1ApproachPose) || timedOut(STATE_TIMEOUT)) {
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
                        setDrivePower(0, 0, 0, 0);
                        shootAllBalls();
                        transitionTo(AutoState.DRIVE_PATH5_APPROACH);
                    }
                    break;

                // ---------------- PATH 5 APPROACH ----------------
                case DRIVE_PATH5_APPROACH:
                    if (stateJustEntered()) {
                        follower.followPath(path5Approach, false);
                    }
                    if (pathComplete(row2ApproachPose) || timedOut(STATE_TIMEOUT)) {
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
                        setDrivePower(0, 0, 0, 0);
                        shootAllBalls();
                        transitionTo(AutoState.DRIVE_PATH8_APPROACH);
                    }
                    break;

                // ---------------- PATH 8 APPROACH ----------------
                case DRIVE_PATH8_APPROACH:
                    if (stateJustEntered()) {
                        follower.followPath(path8Approach, false);
                    }
                    if (pathComplete(row3ApproachPose) || timedOut(STATE_TIMEOUT)) {
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
                        setDrivePower(0, 0, 0, 0);
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

    // ---------------- SERVO SETTER ----------------
    private void servoSetter() {
        double currentPos = gate.getPosition();
        double degreesBack = backNum;
        double totalDegrees = 1800.0;

        double positionChange = degreesBack / totalDegrees;
        double newPos = currentPos - positionChange;

        newPos = Math.max(0.0, Math.min(1.0, newPos));
        gate.setPosition(newPos);
    }

    // ---------------- SHOOTING ROUTINE ----------------
    private void shootAllBalls() throws InterruptedException {

        // OPEN GATE FIRST
        servoSetter();

        shooter.setTargetRPM(2100);
        sleep(1500);

        // Ball 1
        intakeTop.setPower(-1.0);
        sleep(300);
        intakeTop.setPower(0.5);
        sleep(600);
        intakeTop.setPower(0);
        sleep(300);

        // Ball 2
        intakeTop.setPower(-1.0);
        sleep(500);
        intakeTop.setPower(0.5);
        sleep(600);
        intakeTop.setPower(0);
        sleep(300);

        // Ball 3
        intakeTop.setPower(-1.0);
        sleep(800);
        intakeTop.setPower(0);

        shooter.stopShooter();

        // CLOSE GATE AT END
        gate.setPosition(1.0);
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
            new PathConstraints(0.765, 10.2, 0.6375, 0.6375);

    private static final PathConstraints SLOW_CONSTRAINTS =
            new PathConstraints(0.315, 4.2, 0.2625, 0.2625);

    private static final PathConstraints SUPER_SLOW_CONSTRAINTS =
            new PathConstraints(0.135, 1.8, 0.1125, 0.1125);

    // ---------------- BUILD PATHS ----------------
    private void buildPaths() {

        // Path 1A
        path1A = follower.pathBuilder()
                .addPath(new BezierLine(startPose, preShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(-136.8))
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        // Path 1B
        path1B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setConstantHeadingInterpolation(Math.toRadians(-136.8))
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        // Path 1C
        path1C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, launchingPose))
                .setConstantHeadingInterpolation(Math.toRadians(-136.8))
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        // ---------------- ROW 1 ----------------
        path2Approach = follower.pathBuilder()
                .addPath(new BezierLine(launchingPose, row1ApproachPose))
                .setLinearHeadingInterpolation(launchingPose.getHeading(), 0)
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(row1ApproachPose, path2Pose))
                .setConstantHeadingInterpolation(0)
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(path2Pose, path3Pose))
                .setConstantHeadingInterpolation(0)
                .setConstraints(SUPER_SLOW_CONSTRAINTS)
                .build();

        // ---------------- ROW 2 ----------------
        path5Approach = follower.pathBuilder()
                .addPath(new BezierLine(launchingPose, row2ApproachPose))
                .setLinearHeadingInterpolation(launchingPose.getHeading(), 0)
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(row2ApproachPose, path5Pose))
                .setConstantHeadingInterpolation(0)
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(path5Pose, path6Pose))
                .setConstantHeadingInterpolation(0)
                .setConstraints(SUPER_SLOW_CONSTRAINTS)
                .build();

        // ---------------- ROW 3 ----------------
        path8Approach = follower.pathBuilder()
                .addPath(new BezierLine(launchingPose, row3ApproachPose))
                .setLinearHeadingInterpolation(launchingPose.getHeading(), 0)
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(row3ApproachPose, path8Pose))
                .setConstantHeadingInterpolation(0)
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(path8Pose, path9Pose))
                .setConstantHeadingInterpolation(0)
                .setConstraints(SUPER_SLOW_CONSTRAINTS)
                .build();

        // ---------------- RETURN TO SHOOT ----------------
        path4A = follower.pathBuilder()
                .addPath(new BezierLine(path3Pose, preShootPose))
                .setLinearHeadingInterpolation(0, Math.toRadians(-136.8))
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        path4B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setConstantHeadingInterpolation(Math.toRadians(-136.8))
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        path4C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, launchingPose))
                .setConstantHeadingInterpolation(Math.toRadians(-136.8))
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        // ---------------- RETURN 2 ----------------
        path7A = follower.pathBuilder()
                .addPath(new BezierLine(path6Pose, preShootPose))
                .setLinearHeadingInterpolation(0, Math.toRadians(-136.8))
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        path7B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setConstantHeadingInterpolation(Math.toRadians(-136.8))
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        path7C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, launchingPose))
                .setConstantHeadingInterpolation(Math.toRadians(-136.8))
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        // ---------------- RETURN 3 ----------------
        path10A = follower.pathBuilder()
                .addPath(new BezierLine(path9Pose, preShootPose))
                .setLinearHeadingInterpolation(0, Math.toRadians(-136.8))
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        path10B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setConstantHeadingInterpolation(Math.toRadians(-136.8))
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        path10C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, launchingPose))
                .setConstantHeadingInterpolation(Math.toRadians(-136.8))
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        // ---------------- FINAL LEAVE ----------------
        path11 = follower.pathBuilder()
                .addPath(new BezierLine(launchingPose, path11Pose))
                .setLinearHeadingInterpolation(Math.toRadians(-136.8), 0)
                .setConstraints(FAST_CONSTRAINTS)
                .build();
    }
}