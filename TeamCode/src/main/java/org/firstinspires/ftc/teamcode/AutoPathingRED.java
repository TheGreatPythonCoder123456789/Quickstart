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
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name="AutonPathingRED", group="TeleOp")
public class AutoPathingRED extends OpMode {

    private enum AutoState {
        DRIVE_PATH1,
        FIRE_BALLS,
        DRIVE_PATH2,
        DRIVE_PATH3,
        DRIVE_PATH4,
        FIRE_BALLS1,
        DRIVE_PATH5,
        DRIVE_PATH6,
        DRIVE_PATH7,
        FIRE_BALLS2,
        DRIVE_PATH8,
        DRIVE_PATH9,
        DRIVE_PATH10,
        FIRE_BALLS3,
        DRIVE_PATH11,
        DONE
    }

    private Follower follower;
    private Timer timer;

    private DcMotor intakeTop;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private ShooterSubsystem shooter;

    private final Pose startPose = new Pose(87, 9, Math.toRadians(270));
    private final Pose path1Pose = new Pose(84, 99, Math.toRadians(-149));
    private final Pose path2Pose = new Pose(100, 84, Math.toRadians(0));
    private final Pose path3Pose = new Pose(129, 84, Math.toRadians(0));
    private final Pose path4Pose = new Pose(84, 99, Math.toRadians(-149));
    private final Pose path5Pose = new Pose(100,60,Math.toRadians(0));
    private final Pose path6Pose = new Pose(129,60,Math.toRadians(0));
    private final Pose path7Pose = new Pose(84,99,Math.toRadians(-149));
    private final Pose path8Pose = new Pose(100,36,Math.toRadians(0));
    private final Pose path9Pose = new Pose(129,36,Math.toRadians(0));
    private final Pose path10Pose = new Pose(84,99,Math.toRadians(-149));
    private final Pose path11Pose = new Pose(109,71,Math.toRadians(0));


    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11;

    private AutoState currentState;
    private boolean pathStarted = false;

    @Override
    public void init() {
        timer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        intakeTop = hardwareMap.get(DcMotor.class, "intakeTop");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        intakeTop.setDirection(DcMotor.Direction.FORWARD);
        intakeTop.setPower(0);

        shooter = new ShooterSubsystem(hardwareMap);

        setDrivePower(0, 0, 0, 0);

        buildPaths();
        follower.setPose(startPose);
        currentState = AutoState.DRIVE_PATH1;
    }

    private void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, path1Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), path1Pose.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(path1Pose, path2Pose))
                .setLinearHeadingInterpolation(path1Pose.getHeading(), path2Pose.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(path2Pose, path3Pose))
                .setLinearHeadingInterpolation(path2Pose.getHeading(), path3Pose.getHeading())
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(path3Pose, path4Pose))
                .setLinearHeadingInterpolation(path3Pose.getHeading(), path4Pose.getHeading())
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(path4Pose, path5Pose))
                .setLinearHeadingInterpolation(path4Pose.getHeading(), path5Pose.getHeading())
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(path5Pose, path6Pose))
                .setLinearHeadingInterpolation(path5Pose.getHeading(), path6Pose.getHeading())
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(path6Pose, path7Pose))
                .setLinearHeadingInterpolation(path6Pose.getHeading(), path7Pose.getHeading())
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(path7Pose, path8Pose))
                .setLinearHeadingInterpolation(path7Pose.getHeading(), path8Pose.getHeading())
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(path8Pose, path9Pose))
                .setLinearHeadingInterpolation(path8Pose.getHeading(), path9Pose.getHeading())
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierLine(path9Pose, path10Pose))
                .setLinearHeadingInterpolation(path9Pose.getHeading(), path10Pose.getHeading())
                .build();

        path11 = follower.pathBuilder()
                .addPath(new BezierLine(path10Pose, path11Pose))
                .setLinearHeadingInterpolation(path10Pose.getHeading(), path11Pose.getHeading())
                .build();
    }

    private void setDrivePower(double lf, double rf, double lb, double rb) {
        frontLeft.setPower(lf);
        frontRight.setPower(rf);
        backLeft.setPower(lb);
        backRight.setPower(rb);
    }

    @Override
    public void loop() {
        follower.update();

        switch (currentState) {

            case DRIVE_PATH1:
                if (!pathStarted) {
                    follower.followPath(path1, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    shooter.setTargetRPM(2100);   // spin up shooter
                    sleep(2000);                  // wait 2 seconds
                    currentState = AutoState.FIRE_BALLS;
                }
                break;

            case FIRE_BALLS:
                try {
                    // Ball 1
                    intakeTop.setPower(-1.0);
                    sleep(500);
                    intakeTop.setPower(0);
                    sleep(500);

                    // Ball 2
                    intakeTop.setPower(-1.0);
                    sleep(800);
                    intakeTop.setPower(0);
                    sleep(500);

                    // Ball 3
                    intakeTop.setPower(-1.0);
                    sleep(1000);
                    intakeTop.setPower(0);

                    // Shooter continues for 300 ms after last ball
                    sleep(300);
                    shooter.stopShooter();
                    intakeTop.setPower(0);

                } catch (Exception e) {
                    telemetry.addLine("Error in shooting sequence: " + e.getMessage());
                }

                currentState = AutoState.DRIVE_PATH2;
                break;

            case DRIVE_PATH2:
                if (!pathStarted) {
                    follower.followPath(path2, true);
                    pathStarted = true;
                    intakeTop.setPower(-1.0); // pickup
                }
                if (!follower.isBusy()) {
                    intakeTop.setPower(0);
                    pathStarted = false;
                    currentState = AutoState.DRIVE_PATH3;
                }
                break;

            case DRIVE_PATH3:
                if (!pathStarted) {
                    follower.followPath(path3, true);
                    pathStarted = true;
                    intakeTop.setPower(-1.0); // pickup
                }
                if (!follower.isBusy()) {
                    intakeTop.setPower(0);
                    currentState = AutoState.DONE;
                }
                break;

            case DRIVE_PATH4:
                if (!pathStarted) {
                    follower.followPath(path4, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    currentState = AutoState.DONE;
                }
                break;

            case FIRE_BALLS1:
                try {
                    // Ball 1
                    intakeTop.setPower(-1.0);
                    sleep(500);
                    intakeTop.setPower(0);
                    sleep(500);

                    // Ball 2
                    intakeTop.setPower(-1.0);
                    sleep(800);
                    intakeTop.setPower(0);
                    sleep(500);

                    // Ball 3
                    intakeTop.setPower(-1.0);
                    sleep(1000);
                    intakeTop.setPower(0);

                    // Shooter continues for 300 ms after last ball
                    sleep(300);
                    shooter.stopShooter();
                    intakeTop.setPower(0);

                } catch (Exception e) {
                    telemetry.addLine("Error in shooting sequence: " + e.getMessage());
                }

                currentState = AutoState.DRIVE_PATH4;
                break;

            case DRIVE_PATH5:
                if (!pathStarted) {
                    follower.followPath(path5, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    currentState = AutoState.DONE;
                }
                break;

            case DRIVE_PATH6:
                if (!pathStarted) {
                    follower.followPath(path6, true);
                    pathStarted = true;
                    intakeTop.setPower(-1.0); // pickup
                }
                if (!follower.isBusy()) {
                    intakeTop.setPower(0);
                    currentState = AutoState.DONE;
                }
                break;

            case DRIVE_PATH7:
                if (!pathStarted) {
                    follower.followPath(path7, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    currentState = AutoState.DONE;
                }
                break;

            case FIRE_BALLS2:
                try {
                    // Ball 1
                    intakeTop.setPower(-1.0);
                    sleep(500);
                    intakeTop.setPower(0);
                    sleep(500);

                    // Ball 2
                    intakeTop.setPower(-1.0);
                    sleep(800);
                    intakeTop.setPower(0);
                    sleep(500);

                    // Ball 3
                    intakeTop.setPower(-1.0);
                    sleep(1000);
                    intakeTop.setPower(0);

                    // Shooter continues for 300 ms after last ball
                    sleep(300);
                    shooter.stopShooter();
                    intakeTop.setPower(0);

                } catch (Exception e) {
                    telemetry.addLine("Error in shooting sequence: " + e.getMessage());
                }

                currentState = AutoState.DRIVE_PATH7;
                break;

            case DRIVE_PATH8:
                if (!pathStarted) {
                    follower.followPath(path8, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    currentState = AutoState.DONE;
                }
                break;

            case DRIVE_PATH9:
                if (!pathStarted) {
                    follower.followPath(path9, true);
                    pathStarted = true;
                    intakeTop.setPower(-1.0); // pickup
                }
                if (!follower.isBusy()) {
                    intakeTop.setPower(0);
                    currentState = AutoState.DONE;
                }
                break;

            case DRIVE_PATH10:
                if (!pathStarted) {
                    follower.followPath(path10, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    currentState = AutoState.DONE;
                }
                break;

            case FIRE_BALLS3:
                try {
                    // Ball 1
                    intakeTop.setPower(-1.0);
                    sleep(500);
                    intakeTop.setPower(0);
                    sleep(500);

                    // Ball 2
                    intakeTop.setPower(-1.0);
                    sleep(800);
                    intakeTop.setPower(0);
                    sleep(500);

                    // Ball 3
                    intakeTop.setPower(-1.0);
                    sleep(1000);
                    intakeTop.setPower(0);

                    // Shooter continues for 300 ms after last ball
                    sleep(300);
                    shooter.stopShooter();
                    intakeTop.setPower(0);

                } catch (Exception e) {
                    telemetry.addLine("Error in shooting sequence: " + e.getMessage());
                }

                currentState = AutoState.DRIVE_PATH10;
                break;

            case DRIVE_PATH11:
                if (!pathStarted) {
                    follower.followPath(path11, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    currentState = AutoState.DONE;
                }
                break;

            case DONE:
                shooter.stopShooter();
                intakeTop.setPower(0);
                setDrivePower(0,0,0,0);
                telemetry.addLine("Autonomous Complete");
                break;
        }

        telemetry.addData("State", currentState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Intake Power", intakeTop.getPower());
        telemetry.addData("Shooter Left Vel", shooter.getLeftShooterVelocity());
        telemetry.addData("Shooter Right Vel", shooter.getRightShooterVelocity());
    }

    // Helper sleep method since OpMode doesn't have Thread.sleep
    private void sleep(long millis) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < millis) {
            follower.update();   // keep pathing alive
            telemetry.update();  // optional
        }
    }
}