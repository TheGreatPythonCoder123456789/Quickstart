package org.firstinspires.ftc.teamcode.OmarMingzhe.Autonomous;

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
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemCloseShooting;

@Autonomous(name = "Blue Back CURVED FINAL", group = "Autonomous")
public class BlueBackCurved extends LinearOpMode {

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

    private Follower follower;
    private Timer stateTimer;

    private DcMotor intakeTop;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private ShooterSubsystemCloseShooting shooter;
    private Servo gate;

    private double backNum = 80;
    private double RPMlow = 1020; //1885

    private final Pose startPose = new Pose(56, 9, Math.toRadians(270));

    private final Pose preShootPose  = new Pose(57.55, 77.24, Math.toRadians(-43.2));
    private final Pose midShootPose  = new Pose(57.55, 86.84, Math.toRadians(-39));
    private final Pose launchingPose = new Pose(57.55, 89.84, Math.toRadians(-35));

    private final Pose row1ApproachPose = new Pose(52, 94, Math.toRadians(180));
    private final Pose row2ApproachPose = new Pose(52, 70, Math.toRadians(180));
    private final Pose row3ApproachPose = new Pose(52, 46, Math.toRadians(180));

    private final Pose path2Pose = new Pose(52, 84, Math.toRadians(180));
    private final Pose path3Pose = new Pose(15, 84, Math.toRadians(180));

    private final Pose path5Pose = new Pose(52, 60, Math.toRadians(180));
    private final Pose path6Pose = new Pose(9, 60, Math.toRadians(180));

    private final Pose path8Pose = new Pose(52, 36, Math.toRadians(180));
    private final Pose path9Pose = new Pose(9, 36, Math.toRadians(180));

    private PathChain path1A, path1B, path1C;
    private PathChain path2Approach, path2, path3;
    private PathChain path4A, path4B, path4C;
    private PathChain path5Approach, path5, path6;
    private PathChain path7A, path7B, path7C;
    private PathChain path8Approach, path8, path9;

    private AutoState currentState;
    private AutoState lastState = null;

    private static final PathConstraints FAST =
            new PathConstraints(0.765, 10.2, 0.6375, 0.6375);

    private static final PathConstraints SLOW =
            new PathConstraints(0.315, 4.2, 0.2625, 0.2625);

    private static final PathConstraints SUPER_SLOW =
            new PathConstraints(0.25, 3.5, 0.2, 0.2);

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

        buildPaths();

        follower.setPose(startPose);
        currentState = AutoState.DRIVE_PATH1A;

        waitForStart();

        while (opModeIsActive() && currentState != AutoState.DONE) {

            follower.update();

            switch (currentState) {

                case DRIVE_PATH1A:
                    if (entered()) {
                        shooter.setTargetRPM(RPMlow);
                        follower.followPath(path1A, false);
                    }
                    if (!follower.isBusy()) next(AutoState.DRIVE_PATH1B);
                    break;

                case DRIVE_PATH1B:
                    if (entered()) follower.followPath(path1B, false);
                    if (!follower.isBusy()) next(AutoState.DRIVE_PATH1C);
                    break;

                case DRIVE_PATH1C:
                    if (entered()) follower.followPath(path1C, false);
                    if (!follower.isBusy()) next(AutoState.FIRE_BALLS);
                    break;

                case FIRE_BALLS:
                    if (entered()) {
                        follower.breakFollowing();
                        stopDrive();
                        shootAllBalls();
                        next(AutoState.DRIVE_PATH2_APPROACH);
                    }
                    break;

                case DRIVE_PATH2_APPROACH:
                    if (entered()) follower.followPath(path2Approach, false);
                    if (!follower.isBusy()) next(AutoState.DRIVE_PATH2);
                    break;

                case DRIVE_PATH2:
                    if (entered()) {
                        follower.followPath(path2, false);
                        intakeTop.setPower(-1);
                    }
                    if (!follower.isBusy()) {
                        intakeTop.setPower(0);
                        next(AutoState.DRIVE_PATH3);
                    }
                    break;

                case DRIVE_PATH3:
                    if (entered()) {
                        follower.followPath(path3, false);
                        intakeTop.setPower(-1);
                    }
                    if (!follower.isBusy()) {
                        intakeTop.setPower(0);
                        next(AutoState.DRIVE_PATH4A);
                    }
                    break;

                case DRIVE_PATH4A:
                    if (entered()) {
                        shooter.setTargetRPM(RPMlow);
                        follower.followPath(path4A, false);
                    }
                    if (!follower.isBusy()) next(AutoState.DRIVE_PATH4B);
                    break;

                case DRIVE_PATH4B:
                    if (entered()) follower.followPath(path4B, false);
                    if (!follower.isBusy()) next(AutoState.DRIVE_PATH4C);
                    break;

                case DRIVE_PATH4C:
                    if (entered()) follower.followPath(path4C, false);
                    if (!follower.isBusy()) next(AutoState.FIRE_BALLS1);
                    break;

                case FIRE_BALLS1:
                    if (entered()) {
                        follower.breakFollowing();
                        stopDrive();
                        shootAllBalls();
                        next(AutoState.DRIVE_PATH5_APPROACH);
                    }
                    break;

                case DRIVE_PATH5_APPROACH:
                    if (entered()) follower.followPath(path5Approach, false);
                    if (!follower.isBusy()) next(AutoState.DRIVE_PATH5);
                    break;

                case DRIVE_PATH5:
                    if (entered()) {
                        follower.followPath(path5, false);
                        intakeTop.setPower(-1);
                    }
                    if (!follower.isBusy()) {
                        intakeTop.setPower(0);
                        next(AutoState.DRIVE_PATH6);
                    }
                    break;

                case DRIVE_PATH6:
                    if (entered()) {
                        follower.followPath(path6, false);
                        intakeTop.setPower(-1);
                    }
                    if (!follower.isBusy()) {
                        intakeTop.setPower(0);
                        next(AutoState.DRIVE_PATH7A);
                    }
                    break;

                case DRIVE_PATH7A:
                    if (entered()) {
                        shooter.setTargetRPM(RPMlow);
                        follower.followPath(path7A, false);
                    }
                    if (!follower.isBusy()) next(AutoState.DRIVE_PATH7B);
                    break;

                case DRIVE_PATH7B:
                    if (entered()) follower.followPath(path7B, false);
                    if (!follower.isBusy()) next(AutoState.DRIVE_PATH7C);
                    break;

                case DRIVE_PATH7C:
                    if (entered()) follower.followPath(path7C, false);
                    if (!follower.isBusy()) next(AutoState.FIRE_BALLS2);
                    break;

                case FIRE_BALLS2:
                    if (entered()) {
                        follower.breakFollowing();
                        stopDrive();
                        shootAllBalls();
                        next(AutoState.DRIVE_PATH8_APPROACH);
                    }
                    break;

                case DRIVE_PATH8_APPROACH:
                    if (entered()) follower.followPath(path8Approach, false);
                    if (!follower.isBusy()) next(AutoState.DRIVE_PATH8);
                    break;

                case DRIVE_PATH8:
                    if (entered()) {
                        follower.followPath(path8, false);
                        intakeTop.setPower(-1);
                    }
                    if (!follower.isBusy()) {
                        intakeTop.setPower(0);
                        next(AutoState.DRIVE_PATH9);
                    }
                    break;

                case DRIVE_PATH9:
                    if (entered()) {
                        follower.followPath(path9, false);
                        intakeTop.setPower(-1);
                    }
                    if (!follower.isBusy()) {
                        intakeTop.setPower(0);
                        next(AutoState.DONE);
                    }
                    break;
            }
        }

        shooter.stopShooter();
        intakeTop.setPower(0);
        stopDrive();
    }

    private boolean entered() {
        if (lastState != currentState) {
            lastState = currentState;
            return true;
        }
        return false;
    }

    private void next(AutoState newState) {
        currentState = newState;
        stateTimer.resetTimer();
    }

    private void servoSetter() {
        double currentPos = gate.getPosition();
        double positionChange = backNum / 1800.0;
        double newPos = Math.max(0.0, Math.min(1.0, currentPos - positionChange));
        gate.setPosition(newPos);
    }

    private void waitForShooterReady() {
        long start = System.currentTimeMillis();
        while (opModeIsActive()) {
            if (shooter.getLeftShooterVelocity() > 880 &&
                    shooter.getRightShooterVelocity() > 880) break;
            if (System.currentTimeMillis() - start > 1200) break;
            sleep(10);
        }
    }

    private void shootAllBalls() throws InterruptedException {
        servoSetter();
        waitForShooterReady();
        intakeTop.setPower(-1);
        sleep(250);
        intakeTop.setPower(1);
        sleep(150);
        intakeTop.setPower(0);
        sleep(733);

        intakeTop.setPower(-1);
        sleep(350);
        intakeTop.setPower(1);
        sleep(150);
        intakeTop.setPower(0);
        sleep(533);

        intakeTop.setPower(-1);
        sleep(700);

        intakeTop.setPower(0);
        shooter.stopShooter();
        gate.setPosition(1.0);
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void buildPaths() {
        path1A = follower.pathBuilder()
                .addPath(new BezierLine(startPose, preShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), preShootPose.getHeading())
                .setConstraints(FAST)
                .build();

        path1B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setLinearHeadingInterpolation(preShootPose.getHeading(), midShootPose.getHeading())
                .setConstraints(SLOW)
                .build();

        path1C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, launchingPose))
                .setLinearHeadingInterpolation(midShootPose.getHeading(), launchingPose.getHeading())
                .setConstraints(SLOW)
                .build();

        path2Approach = follower.pathBuilder()
                .addPath(new BezierCurve(
                        launchingPose,
                        new Pose(60, 96, launchingPose.getHeading()),
                        new Pose(50, 102, Math.toRadians(180)),
                        row1ApproachPose))
                .setLinearHeadingInterpolation(launchingPose.getHeading(), row1ApproachPose.getHeading())
                .setConstraints(SLOW)
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(row1ApproachPose, path2Pose))
                .setLinearHeadingInterpolation(row1ApproachPose.getHeading(), path2Pose.getHeading())
                .setConstraints(SLOW)
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(path2Pose, path3Pose))
                .setLinearHeadingInterpolation(path2Pose.getHeading(), path3Pose.getHeading())
                .setConstraints(SUPER_SLOW)
                .build();

        path4A = follower.pathBuilder()
                .addPath(new BezierCurve(
                        path3Pose,
                        new Pose(28, 95, Math.toRadians(180)),
                        new Pose(48, 88, Math.toRadians(-43.2)),
                        preShootPose))
                .setLinearHeadingInterpolation(path3Pose.getHeading(), preShootPose.getHeading())
                .setConstraints(SUPER_SLOW)
                .build();

        path4B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setLinearHeadingInterpolation(preShootPose.getHeading(), midShootPose.getHeading())
                .setConstraints(SLOW)
                .build();

        path4C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, launchingPose))
                .setLinearHeadingInterpolation(midShootPose.getHeading(), launchingPose.getHeading())
                .setConstraints(SLOW)
                .build();

        path5Approach = follower.pathBuilder()
                .addPath(new BezierCurve(
                        launchingPose,
                        new Pose(60, 74, launchingPose.getHeading()),
                        new Pose(50, 80, Math.toRadians(180)),
                        row2ApproachPose))
                .setLinearHeadingInterpolation(launchingPose.getHeading(), Math.toRadians(180))
                .setConstraints(SLOW)
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(row2ApproachPose, path5Pose))
                .setLinearHeadingInterpolation(row2ApproachPose.getHeading(), path5Pose.getHeading())
                .setConstraints(SUPER_SLOW)
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(path5Pose, path6Pose))
                .setLinearHeadingInterpolation(path5Pose.getHeading(), path6Pose.getHeading())
                .setConstraints(SUPER_SLOW)
                .build();

        path7A = follower.pathBuilder()
                .addPath(new BezierLine(path6Pose, preShootPose))
                .setLinearHeadingInterpolation(path6Pose.getHeading(), preShootPose.getHeading())
                .setConstraints(SUPER_SLOW)
                .build();

        path7B = follower.pathBuilder()
                .addPath(new BezierLine(preShootPose, midShootPose))
                .setLinearHeadingInterpolation(preShootPose.getHeading(), midShootPose.getHeading())
                .setConstraints(SLOW)
                .build();

        path7C = follower.pathBuilder()
                .addPath(new BezierLine(midShootPose, launchingPose))
                .setLinearHeadingInterpolation(midShootPose.getHeading(), launchingPose.getHeading())
                .setConstraints(SLOW)
                .build();

        path8Approach = follower.pathBuilder()
                .addPath(new BezierCurve(
                        launchingPose,
                        new Pose(60, 55, launchingPose.getHeading()),
                        new Pose(50, 60, Math.toRadians(180)),
                        row3ApproachPose))
                .setLinearHeadingInterpolation(launchingPose.getHeading(), Math.toRadians(180))
                .setConstraints(SLOW)
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(row3ApproachPose, path8Pose))
                .setLinearHeadingInterpolation(row3ApproachPose.getHeading(), path8Pose.getHeading())
                .setConstraints(SLOW)
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(path8Pose, path9Pose))
                .setLinearHeadingInterpolation(path8Pose.getHeading(), path9Pose.getHeading())
                .setConstraints(SUPER_SLOW)
                .build();
    }
}
