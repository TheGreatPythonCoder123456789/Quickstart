package org.firstinspires.ftc.teamcode.OmarMingza.Autonomous;

// remove intake approach poses and make them use old ones.
// because problem is now bump under robot
//should be best one if this works.

// ---------------- Imports ----------------
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
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemCloseShooting;

@Autonomous(name = "Blue Back Meet 4", group = "Autonomous")
public class BlueBackMeet4 extends LinearOpMode {

    // ---------------- State Machine ----------------
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
        DONE
    }

    // ---------------- Core Objects ----------------
    private Follower follower;
    private Timer stateTimer;

    // ---------------- Hardware ----------------
    private DcMotor intakeTop;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private ShooterSubsystemCloseShooting shooter;
    private Servo gate;

    // ---------------- Shooter Config ----------------
    private double backNum = 80;

    // ---------------- Mirrored Poses ----------------
    // start heading NOT mirrored
    private final Pose startPose = new Pose(56, 9, Math.toRadians(270));

    private final Pose preShootPose  = new Pose(57.55, 77.24, Math.toRadians(-43.2)); //-43.2
    private final Pose midShootPose  = new Pose(57.55, 86.84, Math.toRadians(-39)); //-42
    private final Pose launchingPose = new Pose(57.55, 89.84, Math.toRadians(-35)); //-40

    private final Pose row1ApproachPose = new Pose(52, 94, Math.toRadians(180));
    private final Pose row2ApproachPose = new Pose(52, 70, Math.toRadians(180));
    private final Pose row3ApproachPose = new Pose(52, 46, Math.toRadians(180));

    private final Pose path2Pose = new Pose(52, 84, Math.toRadians(180));
    private final Pose path3Pose = new Pose(15, 84, Math.toRadians(180));

    private final Pose path5Pose = new Pose(52, 60, Math.toRadians(180));
    private final Pose path6Pose = new Pose(9, 60, Math.toRadians(180));

    private final Pose path8Pose = new Pose(52, 36, Math.toRadians(180));
    private final Pose path9Pose = new Pose(9, 36, Math.toRadians(180));

    // ---------------- PathChains ----------------
    private PathChain path1A, path1B, path1C;
    private PathChain path2Approach, path2, path3;
    private PathChain path5Approach, path5, path6;
    private PathChain path8Approach, path8, path9;
    private PathChain path4A, path4B, path4C;
    private PathChain path7A, path7B, path7C;

    // ---------------- State Tracking ----------------
    private AutoState currentState;
    private AutoState lastState = null;

    // ---------------- Constants ----------------
    private static final double POSE_TOLERANCE = 3.5;
    private static final double STATE_TIMEOUT = 3.0;

    private double RPMshot = 2025;
    private double RPMlow = 2025;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        stateTimer = new Timer();

        intakeTop = hardwareMap.get(DcMotor.class, "intakeTop");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        gate = hardwareMap.get(Servo.class, "gate");
        gate.setPosition(1.0);

        shooter = new ShooterSubsystemCloseShooting(hardwareMap);

        intakeTop.setPower(0);
        setDrivePower(0, 0, 0, 0);

        buildPaths();

        follower.setPose(startPose);
        currentState = AutoState.DRIVE_PATH1A;

        waitForStart();
        stateTimer.resetTimer();

        // ---------------- Main Loop ----------------
        while (opModeIsActive() && currentState != AutoState.DONE) {

            follower.update();

            switch (currentState) {

                case DRIVE_PATH1A:
                    if (stateJustEntered()) {
                        shooter.setTargetRPM(RPMlow);
                        follower.followPath(path1A, false);
                    }
                    if (pathComplete(preShootPose) || timedOut(STATE_TIMEOUT))
                        transitionTo(AutoState.DRIVE_PATH1B);
                    break;

                case DRIVE_PATH1B:
                    if (stateJustEntered())
                        follower.followPath(path1B, false);
                    if (pathComplete(midShootPose) || timedOut(STATE_TIMEOUT))
                        transitionTo(AutoState.DRIVE_PATH1C);
                    break;

                case DRIVE_PATH1C:
                    if (stateJustEntered())
                        follower.followPath(path1C, false);
                    if (pathComplete(launchingPose) || timedOut(STATE_TIMEOUT))
                        transitionTo(AutoState.FIRE_BALLS);
                    break;

                case FIRE_BALLS:
                    if (stateJustEntered()) {
                        follower.breakFollowing();
                        setDrivePower(0, 0, 0, 0);
                        shooter.setTargetRPM(RPMlow);
                        shootAllBalls();
                        transitionTo(AutoState.DRIVE_PATH2_APPROACH);
                    }
                    break;

                case DRIVE_PATH2_APPROACH:
                    if (stateJustEntered())
                        follower.followPath(path2Approach, false);
                    if (pathComplete(row1ApproachPose) || timedOut(STATE_TIMEOUT))
                        transitionTo(AutoState.DRIVE_PATH2);
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
                    if (pathComplete(preShootPose) || timedOut(STATE_TIMEOUT))
                        transitionTo(AutoState.DRIVE_PATH4B);
                    break;

                case DRIVE_PATH4B:
                    if (stateJustEntered())
                        follower.followPath(path4B, false);
                    if (pathComplete(midShootPose) || timedOut(STATE_TIMEOUT))
                        transitionTo(AutoState.DRIVE_PATH4C);
                    break;

                case DRIVE_PATH4C:
                    if (stateJustEntered())
                        follower.followPath(path4C, false);
                    if (pathComplete(launchingPose) || timedOut(STATE_TIMEOUT))
                        transitionTo(AutoState.FIRE_BALLS1);
                    break;

                case FIRE_BALLS1:
                    if (stateJustEntered()) {
                        follower.breakFollowing();
                        setDrivePower(0, 0, 0, 0);
                        shooter.setTargetRPM(RPMlow);
                        shootAllBalls();
                        transitionTo(AutoState.DRIVE_PATH5_APPROACH);
                    }
                    break;

                case DRIVE_PATH5_APPROACH:
                    if (stateJustEntered())
                        follower.followPath(path5Approach, false);
                    if (pathComplete(row2ApproachPose) || timedOut(STATE_TIMEOUT))
                        transitionTo(AutoState.DRIVE_PATH5);
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
                    if (pathComplete(preShootPose) || timedOut(STATE_TIMEOUT))
                        transitionTo(AutoState.DRIVE_PATH7B);
                    break;

                case DRIVE_PATH7B:
                    if (stateJustEntered())
                        follower.followPath(path7B, false);
                    if (pathComplete(midShootPose) || timedOut(STATE_TIMEOUT))
                        transitionTo(AutoState.DRIVE_PATH7C);
                    break;

                case DRIVE_PATH7C:
                    if (stateJustEntered())
                        follower.followPath(path7C, false);
                    if (pathComplete(launchingPose) || timedOut(STATE_TIMEOUT))
                        transitionTo(AutoState.FIRE_BALLS2);
                    break;

                case FIRE_BALLS2:
                    if (stateJustEntered()) {
                        follower.breakFollowing();
                        setDrivePower(0, 0, 0, 0);
                        shooter.setTargetRPM(RPMlow);
                        shootAllBalls();
                        transitionTo(AutoState.DRIVE_PATH8_APPROACH);
                    }
                    break;

                case DRIVE_PATH8_APPROACH:
                    if (stateJustEntered())
                        follower.followPath(path8Approach, false);
                    if (pathComplete(row3ApproachPose) || timedOut(STATE_TIMEOUT))
                        transitionTo(AutoState.DRIVE_PATH8);
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
                        transitionTo(AutoState.DONE);
                    }
                    break;

                case DONE:
                    break;
            }
        }

        shooter.stopShooter();
        intakeTop.setPower(0);
        setDrivePower(0, 0, 0, 0);
    }

    // ---------------- Helpers ----------------
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
        return Math.hypot(current.getX() - target.getX(),
                current.getY() - target.getY()) < POSE_TOLERANCE;
    }

    // ---------------- Gate Servo ----------------
    private void servoSetter() {
        double currentPos = gate.getPosition();
        double positionChange = backNum / 1800.0;
        double newPos = Math.max(0.0, Math.min(1.0, currentPos - positionChange));
        gate.setPosition(newPos);
    }

    // ---------------- Shooter Wait ----------------
    private void waitForShooterReady() {
        long start = System.currentTimeMillis();
        while (opModeIsActive()) {
            if (shooter.getLeftShooterVelocity() > 920 &&
                    shooter.getRightShooterVelocity() > 920) break;
            if (System.currentTimeMillis() - start > 1200) break;
            sleep(10);
        }
    }

    // ---------------- Shooting Routine ----------------
    private void shootAllBalls() throws InterruptedException {
        servoSetter();
        waitForShooterReady();

        intakeTop.setPower(-1.0);
        sleep(250);
        intakeTop.setPower(1);
        sleep(150);
        intakeTop.setPower(0);
        sleep(733);

        intakeTop.setPower(-1.0);
        sleep(250);
        intakeTop.setPower(1);
        sleep(150);
        intakeTop.setPower(0);
        sleep(533);

        intakeTop.setPower(-1.0);
        sleep(800);

        intakeTop.setPower(0);
        shooter.stopShooter();
        gate.setPosition(1.0);
    }

    // ---------------- Drive Power ----------------
    private void setDrivePower(double lf, double rf, double lb, double rb) {
        frontLeft.setPower(lf);
        frontRight.setPower(rf);
        backLeft.setPower(lb);
        backRight.setPower(rb);
    }

    // ---------------- Path Constraints ----------------
    private static final PathConstraints FAST_CONSTRAINTS =
            new PathConstraints(0.765, 10.2, 0.6375, 0.6375);

    private static final PathConstraints SLOW_CONSTRAINTS =
            new PathConstraints(0.315, 4.2, 0.2625, 0.2625);

    private static final PathConstraints SUPER_SLOW_CONSTRAINTS =
            new PathConstraints(0.135, 1.8, 0.1125, 0.1125);

    // ---------------- Build Paths ----------------
    private void buildPaths() {

        path1A = follower.pathBuilder()
                .addPath(new BezierLine(startPose, preShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), preShootPose.getHeading()) //Math.toRadians(-43.2)
                .setConstraints(FAST_CONSTRAINTS)
                .build();

        path1B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setLinearHeadingInterpolation(preShootPose.getHeading(), midShootPose.getHeading()) //Math.toRadians(-42)
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        path1C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, launchingPose))
                .setLinearHeadingInterpolation(midShootPose.getHeading(), launchingPose.getHeading()) //Math.toRadians(-40)
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        path2Approach = follower.pathBuilder()
                .addPath(new BezierLine(launchingPose, row1ApproachPose))
                .setLinearHeadingInterpolation(launchingPose.getHeading(), row1ApproachPose.getHeading()) //Math.toRadians(180)
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(row1ApproachPose, path2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(path2Pose, path3Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setConstraints(SUPER_SLOW_CONSTRAINTS)
                .build();

        path4A = follower.pathBuilder()
                .addPath(new BezierLine(path3Pose, preShootPose))
                .setLinearHeadingInterpolation(path3Pose.getHeading(), preShootPose.getHeading()) //Math.toRadians(-43.2)
                .setConstraints(SUPER_SLOW_CONSTRAINTS)
                .build();

        path4B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setLinearHeadingInterpolation(preShootPose.getHeading(), midShootPose.getHeading()) //Math.toRadians(-42)
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        path4C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, launchingPose))
                .setLinearHeadingInterpolation(midShootPose.getHeading(), launchingPose.getHeading()) //Math.toRadians(-40)
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        path5Approach = follower.pathBuilder()
                .addPath(new BezierLine(launchingPose, row2ApproachPose))
                .setLinearHeadingInterpolation(launchingPose.getHeading(), Math.toRadians(180))
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(row2ApproachPose, path5Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setConstraints(SUPER_SLOW_CONSTRAINTS)
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(path5Pose, path6Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setConstraints(SUPER_SLOW_CONSTRAINTS)
                .build();

        path7A = follower.pathBuilder()
                .addPath(new BezierLine(path6Pose, preShootPose))
                .setLinearHeadingInterpolation(path6Pose.getHeading(), preShootPose.getHeading()) //Math.toRadians(-43.2)
                .setConstraints(SUPER_SLOW_CONSTRAINTS)
                .build();

        path7B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setLinearHeadingInterpolation(preShootPose.getHeading(), midShootPose.getHeading()) //Math.toRadians(-42)
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        path7C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, launchingPose))
                .setLinearHeadingInterpolation(midShootPose.getHeading(), launchingPose.getHeading()) //Math.toRadians(-40)
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        path8Approach = follower.pathBuilder()
                .addPath(new BezierLine(launchingPose, row3ApproachPose))
                .setLinearHeadingInterpolation(launchingPose.getHeading(), Math.toRadians(180))
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(row3ApproachPose, path8Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(path8Pose, path9Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setConstraints(SUPER_SLOW_CONSTRAINTS)
                .build();
    }
}
