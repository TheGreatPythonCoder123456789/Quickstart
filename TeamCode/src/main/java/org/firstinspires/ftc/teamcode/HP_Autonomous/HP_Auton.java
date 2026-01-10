package org.firstinspires.ftc.teamcode.HP_Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "Red Human Player Auto", group = "Autonomous")
public class HP_Auton extends LinearOpMode {

    // ---------------- State Machine ----------------
    private enum AutoState {
        PATH_SHOOT_PRELOAD,
        PATH_FIX_HEADING1,
        PATH_APPROACH_BALLS,
        PATH_GO_SHOOT,
        PATH_FIX_HEADING2,
        FIRE_BALLS,
        DONE
    }

    // ---------------- Core Objects ----------------
    private Follower follower;
    private Timer stateTimer;

    // ---------------- Hardware ----------------
    private DcMotor intakeTop;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private ShooterSubsystem shooter;
    private Servo gate;

    // ---------------- Shooter Config ----------------
    private double backNum = 80;

    // ---------------- Poses From JSON ----------------
    private final Pose startPose = new Pose(88, 9, Math.toRadians(90));
    private final Pose endShootPreload = new Pose(87, 13, Math.toRadians(-119));
    private final Pose approachBallsEnd = new Pose(135, 36, Math.toRadians(0));
    private final Pose goShootEnd = new Pose(87, 13, Math.toRadians(0));

    // ---------------- PathChains ----------------
    private PathChain shootPreload, fixHeading1, approachBalls, goShoot, fixHeading2;

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

        shooter = new ShooterSubsystem(hardwareMap);

        intakeTop.setPower(0);
        setDrivePower(0, 0, 0, 0);

        buildPaths();
        follower.setPose(startPose);

        currentState = AutoState.PATH_SHOOT_PRELOAD;

        waitForStart();
        stateTimer.resetTimer();

        while (opModeIsActive() && currentState != AutoState.DONE) {

            follower.update();

            telemetry.addData("Shooter L", shooter.getLeftShooterVelocity());
            telemetry.addData("Shooter R", shooter.getRightShooterVelocity());
            telemetry.addData("State", currentState);
            telemetry.update();

            switch (currentState) {

                case PATH_SHOOT_PRELOAD:
                    if (stateJustEntered()) {
                        shooter.setTargetRPM(RPMlow);
                        follower.followPath(shootPreload, false);
                    }
                    if (pathComplete(endShootPreload) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.PATH_FIX_HEADING1);
                    }
                    break;

                case PATH_FIX_HEADING1:
                    if (stateJustEntered()) {
                        follower.followPath(fixHeading1, false);
                    }
                    if (pathComplete(endShootPreload) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.PATH_APPROACH_BALLS);
                    }
                    break;

                case PATH_APPROACH_BALLS:
                    if (stateJustEntered()) {
                        follower.followPath(approachBalls, false);
                        intakeTop.setPower(-1.0);
                    }
                    if (pathComplete(approachBallsEnd) || timedOut(STATE_TIMEOUT)) {
                        intakeTop.setPower(0);
                        transitionTo(AutoState.PATH_GO_SHOOT);
                    }
                    break;

                case PATH_GO_SHOOT:
                    if (stateJustEntered()) {
                        follower.followPath(goShoot, false);
                    }
                    if (pathComplete(goShootEnd) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.PATH_FIX_HEADING2);
                    }
                    break;

                case PATH_FIX_HEADING2:
                    if (stateJustEntered()) {
                        follower.followPath(fixHeading2, false);
                    }
                    if (pathComplete(goShootEnd) || timedOut(STATE_TIMEOUT)) {
                        transitionTo(AutoState.FIRE_BALLS);
                    }
                    break;

                case FIRE_BALLS:
                    if (stateJustEntered()) {
                        follower.breakFollowing();
                        setDrivePower(0, 0, 0, 0);
                        shooter.setTargetRPM(RPMlow);
                        shootAllBalls();
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
            if (shooter.getLeftShooterVelocity() > 1800 &&
                    shooter.getRightShooterVelocity() > 1800) break;
            if (System.currentTimeMillis() - start > 2000) break;
            sleep(10);
        }
    }

    // ---------------- Shooting Routine ----------------
    private void shootAllBalls() throws InterruptedException {
        servoSetter();
        waitForShooterReady();

        intakeTop.setPower(-1.0);
        sleep(250);
        intakeTop.setPower(0.8);
        sleep(150);
        intakeTop.setPower(0);
        sleep(733);

        intakeTop.setPower(-1.0);
        sleep(250);
        intakeTop.setPower(0.8);
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

        shootPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(88, 9),
                        new Pose(87.264, 11.942),
                        new Pose(87.215, 12.140),
                        new Pose(87.184, 12.263),
                        new Pose(87.163, 12.348),
                        new Pose(87.143, 12.426),
                        new Pose(87.123, 12.509),
                        new Pose(87.104, 12.585),
                        new Pose(87.085, 12.662),
                        new Pose(87.064, 12.745),
                        new Pose(87.044, 12.822),
                        new Pose(87.024, 12.902),
                        new Pose(87, 13)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(-119))
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        fixHeading1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(87, 13),
                        new Pose(87, 13)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-119), Math.toRadians(0))
                .setConstraints(SUPER_SLOW_CONSTRAINTS)
                .build();

        approachBalls = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(87, 13),
                        new Pose(82, 39.4),
                        new Pose(135, 36)
                ))
                .setLinearHeadingInterpolation(0, 0)
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        goShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(135, 36),
                        new Pose(82, 39.4),
                        new Pose(87, 13)
                ))
                .setLinearHeadingInterpolation(0, 0)
                .setConstraints(SLOW_CONSTRAINTS)
                .build();

        fixHeading2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(87, 13),
                        new Pose(87, 13)
                ))
                .setLinearHeadingInterpolation(0, Math.toRadians(-119))
                .setConstraints(SUPER_SLOW_CONSTRAINTS)
                .build();
    }
}