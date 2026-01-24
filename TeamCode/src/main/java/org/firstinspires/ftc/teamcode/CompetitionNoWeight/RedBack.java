package org.firstinspires.ftc.teamcode.CompetitionNoWeight;
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


@Autonomous(name = "USE THIS red back SR", group = "Autonomous")
public class RedBack extends LinearOpMode {


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


    // ---------------- Poses (Option B applied) ----------------
    private final Pose startPose = new Pose(88, 9, Math.toRadians(270));


    private final Pose preShootPose = new Pose(86.45, 77.24, Math.toRadians(-136.8));   // moved 4 down
    private final Pose midShootPose = new Pose(86.45, 86.84, Math.toRadians(-138));   // moved 4 down
    private final Pose launchingPose = new Pose(86.45, 89.84, Math.toRadians(-140));  // moved 4 down


    private final Pose row1ApproachPose = new Pose(92, 94, Math.toRadians(0)); //92 Y
    private final Pose row2ApproachPose = new Pose(92, 70, Math.toRadians(0)); //68 Y
    private final Pose row3ApproachPose = new Pose(92, 46, Math.toRadians(0)); //44 Y


    private final Pose path2Pose = new Pose(92, 84, Math.toRadians(0));
    private final Pose path3Pose = new Pose(129, 84, Math.toRadians(0));


    private final Pose path5Pose = new Pose(92, 60, Math.toRadians(0));
    private final Pose path6Pose = new Pose(135, 60, Math.toRadians(0));


    private final Pose path8Pose = new Pose(92, 36, Math.toRadians(0));
    private final Pose path9Pose = new Pose(135, 36, Math.toRadians(0));


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


        // ---------------- Init Hardware ----------------
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


        // ---------------- Build Paths ----------------
        buildPaths();


        follower.setPose(startPose);
        currentState = AutoState.DRIVE_PATH1A;


        waitForStart();
        stateTimer.resetTimer();


        // ---------------- Main Loop ----------------
        while (opModeIsActive() && currentState != AutoState.DONE) {


            follower.update();


            telemetry.addData("Shooter L", shooter.getLeftShooterVelocity());
            telemetry.addData("Shooter R", shooter.getRightShooterVelocity());
            telemetry.addData("State", currentState);
            telemetry.update();


            switch (currentState) {


                // ---------------- Path 1A ----------------
                case DRIVE_PATH1A:
                    if (stateJustEntered()) {
                        shooter.setTargetRPM(RPMlow);
                        follower.followPath(path1A, false);
                    }
                    if (pathComplete(preShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH1B);
                    }
                    break;


                // ---------------- Path 1B ----------------
                case DRIVE_PATH1B:
                    if (stateJustEntered()) {
                        follower.followPath(path1B, false);
                    }
                    if (pathComplete(midShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH1C);
                    }
                    break;


                // ---------------- Path 1C ----------------
                case DRIVE_PATH1C:
                    if (stateJustEntered()) {
                        follower.followPath(path1C, false);
                    }
                    if (pathComplete(launchingPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.FIRE_BALLS);
                    }
                    break;


                // ---------------- Fire Preload ----------------
                case FIRE_BALLS:
                    if (stateJustEntered()) {
                        follower.breakFollowing();
                        setDrivePower(0, 0, 0, 0);
                        shooter.setTargetRPM(RPMlow);
                        shootAllBalls();
                        transitionTo(AutoState.DRIVE_PATH2_APPROACH);
                    }
                    break;


                // ---------------- Path 2 Approach ----------------
                case DRIVE_PATH2_APPROACH:
                    if (stateJustEntered()) {
                        follower.followPath(path2Approach, false);
                    }
                    if (pathComplete(row1ApproachPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH2);
                    }
                    break;


                // ---------------- Path 2 ----------------
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


                // ---------------- Path 3 ----------------
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


                // ---------------- Path 4A ----------------
                case DRIVE_PATH4A:
                    if (stateJustEntered()) {
                        shooter.setTargetRPM(RPMshot);
                        follower.followPath(path4A, false);
                    }
                    if (pathComplete(preShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH4B);
                    }
                    break;


                // ---------------- Path 4B ----------------
                case DRIVE_PATH4B:
                    if (stateJustEntered()) {
                        follower.followPath(path4B, false);
                    }
                    if (pathComplete(midShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH4C);
                    }
                    break;


                // ---------------- Path 4C ----------------
                case DRIVE_PATH4C:
                    if (stateJustEntered()) {
                        follower.followPath(path4C, false);
                    }
                    if (pathComplete(launchingPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.FIRE_BALLS1);
                    }
                    break;


                // ---------------- Fire Row 1 ----------------
                case FIRE_BALLS1:
                    if (stateJustEntered()) {
                        follower.breakFollowing();
                        setDrivePower(0, 0, 0, 0);
                        shooter.setTargetRPM(2200);
                        shootAllBalls();
                        transitionTo(AutoState.DRIVE_PATH5_APPROACH);
                    }
                    break;


                // ---------------- Path 5 Approach ----------------
                case DRIVE_PATH5_APPROACH:
                    if (stateJustEntered()) {
                        follower.followPath(path5Approach, false);
                    }
                    if (pathComplete(row2ApproachPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH5);
                    }
                    break;


                // ---------------- Path 5 ----------------
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
                // ---------------- Path 6 ----------------
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


                // ---------------- Path 7A ----------------
                case DRIVE_PATH7A:
                    if (stateJustEntered()) {
                        shooter.setTargetRPM(RPMshot);
                        follower.followPath(path7A, false);
                    }
                    if (pathComplete(preShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH7B);
                    }
                    break;


                // ---------------- Path 7B ----------------
                case DRIVE_PATH7B:
                    if (stateJustEntered()) {
                        follower.followPath(path7B, false);
                    }
                    if (pathComplete(midShootPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH7C);
                    }
                    break;


                // ---------------- Path 7C ----------------
                case DRIVE_PATH7C:
                    if (stateJustEntered()) {
                        follower.followPath(path7C, false);
                    }
                    if (pathComplete(launchingPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.FIRE_BALLS2);
                    }
                    break;


                // ---------------- Fire Row 2 ----------------
                case FIRE_BALLS2:
                    if (stateJustEntered()) {
                        follower.breakFollowing();
                        setDrivePower(0, 0, 0, 0);
                        shooter.setTargetRPM(2200);
                        shootAllBalls();
                        transitionTo(AutoState.DRIVE_PATH8_APPROACH);
                    }
                    break;


                // ---------------- Path 8 Approach ----------------
                case DRIVE_PATH8_APPROACH:
                    if (stateJustEntered()) {
                        follower.followPath(path8Approach, false);
                    }
                    if (pathComplete(row3ApproachPose) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.DRIVE_PATH8);
                    }
                    break;


                // ---------------- Path 8 ----------------
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


                // ---------------- Path 9 ----------------
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


                // ---------------- Done ----------------
                case DONE:
                    break;
            }
        }


        shooter.stopShooter();
        intakeTop.setPower(0);
        setDrivePower(0, 0, 0, 0);
    }


    // ---------------- State Helpers ----------------
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
            if (shooter.getLeftShooterVelocity() > 900 &&
                    shooter.getRightShooterVelocity() > 900) break;
            if (System.currentTimeMillis() - start > 1200) break;
            sleep(10);
        }
    }


    // ---------------- Shooting Routine ----------------
    //2 seconds saved if no arc at the end
    //divide among shooting sequences to save time
    // 2/3=0.6666 seconds each shooting has extra now
    //split that up in 2 needed to save so 0.3333 recovery for first 2 balls(after them).
    //convert 0.3333 seconds to milliseconds. (333.33 milliseconds).
    //save to run (!Done)
    private void shootAllBalls() throws InterruptedException {
        servoSetter();
        waitForShooterReady();


        intakeTop.setPower(-1.0);
        sleep(250);
        intakeTop.setPower(0.8);
        sleep(150);
        intakeTop.setPower(0);
        sleep(733); //added 0.33333 seconds


        intakeTop.setPower(-1.0);
        sleep(250);
        intakeTop.setPower(0.8);
        sleep(150);
        intakeTop.setPower(0);
        sleep(533); //added 0.33333 seconds


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
                .setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(-136.8))
                .setConstraints(FAST_CONSTRAINTS)
                .build();


        path1B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setLinearHeadingInterpolation(preShootPose.getHeading(), Math.toRadians(-138))
                .setConstraints(SLOW_CONSTRAINTS)
                .build();


        path1C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, launchingPose))
                .setLinearHeadingInterpolation(midShootPose.getHeading(), Math.toRadians(-140))
                .setConstraints(SLOW_CONSTRAINTS)
                .build();


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


        path5Approach = follower.pathBuilder()
                .addPath(new BezierLine(launchingPose, row2ApproachPose))
                .setLinearHeadingInterpolation(launchingPose.getHeading(), 0)
                .setConstraints(SLOW_CONSTRAINTS)
                .build();


        path5 = follower.pathBuilder()
                .addPath(new BezierLine(row2ApproachPose, path5Pose))
                .setConstantHeadingInterpolation(0)
                .setConstraints(SUPER_SLOW_CONSTRAINTS)
                .build();


        path6 = follower.pathBuilder()
                .addPath(new BezierLine(path5Pose, path6Pose))
                .setConstantHeadingInterpolation(0)
                .setConstraints(SUPER_SLOW_CONSTRAINTS)
                .build();


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


        path4A = follower.pathBuilder()
                .addPath(new BezierLine(path3Pose, preShootPose))
                .setLinearHeadingInterpolation(path3Pose.getHeading(), Math.toRadians(-136.8))
                .setConstraints(SUPER_SLOW_CONSTRAINTS)
                .build();


        path4B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setLinearHeadingInterpolation(preShootPose.getHeading(), Math.toRadians(-138))
                .setConstraints(SLOW_CONSTRAINTS)
                .build();


        path4C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, launchingPose))
                .setLinearHeadingInterpolation(midShootPose.getHeading(), Math.toRadians(-140))
                .setConstraints(SLOW_CONSTRAINTS)
                .build();


        path7A = follower.pathBuilder()
                .addPath(new BezierLine(path6Pose, preShootPose))
                .setLinearHeadingInterpolation(path6Pose.getHeading(), Math.toRadians(-136.8))
                .setConstraints(SUPER_SLOW_CONSTRAINTS)
                .build();


        path7B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setLinearHeadingInterpolation(preShootPose.getHeading(), Math.toRadians(-138))
                .setConstraints(SLOW_CONSTRAINTS)
                .build();


        path7C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, launchingPose))
                .setLinearHeadingInterpolation(midShootPose.getHeading(), Math.toRadians(-140))
                .setConstraints(SLOW_CONSTRAINTS)
                .build();
    }
}