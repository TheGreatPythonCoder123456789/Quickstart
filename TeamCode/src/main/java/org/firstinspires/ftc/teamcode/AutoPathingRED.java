package org.firstinspires.ftc.teamcode;

// this one is updated to pick up all rows of balls and shoot them

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous(name = "AutonPathingRED", group = "Autonomous")
public class AutoPathingRED extends OpMode {

    // State machine for full auton sequence
    private enum AutoState {
        DRIVE_PATH1,
        FIRE_BALLS,     // first shooting cycle (preload)
        DRIVE_PATH2,
        DRIVE_PATH3,
        DRIVE_PATH4,
        FIRE_BALLS1,    // second shooting cycle (row 1)
        DRIVE_PATH5,
        DRIVE_PATH6,
        DRIVE_PATH7,
        FIRE_BALLS2,    // third shooting cycle (row 2)
        DRIVE_PATH8,
        DRIVE_PATH9,
        DRIVE_PATH10,
        FIRE_BALLS3,    // fourth shooting cycle (row 3)
        DRIVE_PATH11,
        DONE
    }

    private Follower follower;
    private Timer timer;

    private DcMotor intakeTop;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private ShooterSubsystem shooter;

    // Field poses (PedroPathing coordinates, inches, 144x144 field)
    // Robot front is intake; shooter fires backward.
    private final Pose startPose  = new Pose(87,  9,  Math.toRadians(270));
    private final Pose path1Pose  = new Pose(84,  99, Math.toRadians(-149));
    private final Pose path2Pose  = new Pose(100, 84, Math.toRadians(0));
    private final Pose path3Pose  = new Pose(129, 84, Math.toRadians(0));
    private final Pose path4Pose  = new Pose(84,  99, Math.toRadians(-149));
    private final Pose path5Pose  = new Pose(100, 60, Math.toRadians(0));
    private final Pose path6Pose  = new Pose(129, 60, Math.toRadians(0));
    private final Pose path7Pose  = new Pose(84,  99, Math.toRadians(-149));
    private final Pose path8Pose  = new Pose(100, 36, Math.toRadians(0));
    private final Pose path9Pose  = new Pose(129, 36, Math.toRadians(0));
    private final Pose path10Pose = new Pose(84,  99, Math.toRadians(-149));
    private final Pose path11Pose = new Pose(109, 71, Math.toRadians(0)); // exit triangle for RP

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

    // Build all paths using PedroPathing Bezier lines and linear heading interpolation
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

    // Simple helper to set drive power (mainly used to ensure stop in DONE)
    private void setDrivePower(double lf, double rf, double lb, double rb) {
        frontLeft.setPower(lf);
        frontRight.setPower(rf);
        backLeft.setPower(lb);
        backRight.setPower(rb);
    }

    @Override
    public void loop() {
        // Always keep Pedro follower updated
        follower.update();

        switch (currentState) {

            // PATH1 (run shooters) → FIRE_BALLS
            case DRIVE_PATH1:
                if (!pathStarted) {
                    // Start shooter spin-up during path1 so it has time to accelerate
                    shooter.setTargetRPM(2100);
                    follower.followPath(path1, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    currentState = AutoState.FIRE_BALLS;
                }
                break;

            // FIRE_BALLS (first shooting cycle, preloads)
            case FIRE_BALLS:
                fireThreeBalls();          // runs intake sequence + keeps shooter spinning briefly
                shooter.stopShooter();     // stop after first cycle
                currentState = AutoState.DRIVE_PATH2;
                break;

            // PATH2 (pickup) → PATH3
            case DRIVE_PATH2:
                if (!pathStarted) {
                    follower.followPath(path2, true);
                    pathStarted = true;
                    intakeTop.setPower(-1.0); // pickup during path2
                }
                if (!follower.isBusy()) {
                    intakeTop.setPower(0);
                    pathStarted = false;
                    currentState = AutoState.DRIVE_PATH3;
                }
                break;

            // PATH3 (pickup) → PATH4
            case DRIVE_PATH3:
                if (!pathStarted) {
                    follower.followPath(path3, true);
                    pathStarted = true;
                    intakeTop.setPower(-1.0); // pickup during path3
                }
                if (!follower.isBusy()) {
                    intakeTop.setPower(0);
                    pathStarted = false;
                    currentState = AutoState.DRIVE_PATH4;
                }
                break;

            // PATH4 (run shooters) → FIRE_BALLS1
            case DRIVE_PATH4:
                if (!pathStarted) {
                    // Spin up shooter during path4
                    shooter.setTargetRPM(2100);
                    follower.followPath(path4, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    currentState = AutoState.FIRE_BALLS1;
                }
                break;

            // FIRE_BALLS1 (second shooting cycle, row 1)
            case FIRE_BALLS1:
                fireThreeBalls();
                shooter.stopShooter();
                currentState = AutoState.DRIVE_PATH5;
                break;

            // PATH5 (pickup) → PATH6
            case DRIVE_PATH5:
                if (!pathStarted) {
                    follower.followPath(path5, true);
                    pathStarted = true;
                    intakeTop.setPower(-1.0); // pickup during path5
                }
                if (!follower.isBusy()) {
                    intakeTop.setPower(0);
                    pathStarted = false;
                    currentState = AutoState.DRIVE_PATH6;
                }
                break;

            // PATH6 (pickup) → PATH7
            case DRIVE_PATH6:
                if (!pathStarted) {
                    follower.followPath(path6, true);
                    pathStarted = true;
                    intakeTop.setPower(-1.0); // pickup during path6
                }
                if (!follower.isBusy()) {
                    intakeTop.setPower(0);
                    pathStarted = false;
                    currentState = AutoState.DRIVE_PATH7;
                }
                break;

            // PATH7 (run shooters) → FIRE_BALLS2
            case DRIVE_PATH7:
                if (!pathStarted) {
                    // Spin up shooter during path7
                    shooter.setTargetRPM(2100);
                    follower.followPath(path7, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    currentState = AutoState.FIRE_BALLS2;
                }
                break;

            // FIRE_BALLS2 (third shooting cycle, row 2)
            case FIRE_BALLS2:
                fireThreeBalls();
                shooter.stopShooter();
                currentState = AutoState.DRIVE_PATH8;
                break;

            // PATH8 (pickup) → PATH9
            case DRIVE_PATH8:
                if (!pathStarted) {
                    follower.followPath(path8, true);
                    pathStarted = true;
                    intakeTop.setPower(-1.0); // pickup during path8
                }
                if (!follower.isBusy()) {
                    intakeTop.setPower(0);
                    pathStarted = false;
                    currentState = AutoState.DRIVE_PATH9;
                }
                break;

            // PATH9 (pickup) → PATH10
            case DRIVE_PATH9:
                if (!pathStarted) {
                    follower.followPath(path9, true);
                    pathStarted = true;
                    intakeTop.setPower(-1.0); // pickup during path9
                }
                if (!follower.isBusy()) {
                    intakeTop.setPower(0);
                    pathStarted = false;
                    currentState = AutoState.DRIVE_PATH10;
                }
                break;

            // PATH10 (run shooters) → FIRE_BALLS3
            case DRIVE_PATH10:
                if (!pathStarted) {
                    // Spin up shooter during path10
                    shooter.setTargetRPM(2100);
                    follower.followPath(path10, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    currentState = AutoState.FIRE_BALLS3;
                }
                break;

            // FIRE_BALLS3 (fourth shooting cycle, row 3)
            case FIRE_BALLS3:
                fireThreeBalls();
                shooter.stopShooter();
                currentState = AutoState.DRIVE_PATH11;
                break;

            // PATH11 (exit triangle for RP) → DONE
            case DRIVE_PATH11:
                if (!pathStarted) {
                    follower.followPath(path11, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    currentState = AutoState.DONE;
                }
                break;

            // Final state: stop everything and sit still
            case DONE:
                shooter.stopShooter();
                intakeTop.setPower(0);
                setDrivePower(0, 0, 0, 0);
                telemetry.addLine("Autonomous Complete");
                break;
        }

        // Telemetry for debugging and tuning
        telemetry.addData("State", currentState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Intake Power", intakeTop.getPower());
        telemetry.addData("Shooter Left Vel", shooter.getLeftShooterVelocity());
        telemetry.addData("Shooter Right Vel", shooter.getRightShooterVelocity());
        telemetry.update();
    }

    /**
     * Helper shooting routine:
     * - Feeds 3 balls with your exact timing pattern.
     * - Shooter is assumed to already be at speed when this is called.
     * - Shooter is NOT stopped here; caller decides when to stop.
     */
    private void fireThreeBalls() {
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

        } catch (Exception e) {
            telemetry.addLine("Error in shooting sequence: " + e.getMessage());
        }
    }

    // Helper sleep method since OpMode doesn't have Thread.sleep
    // Keeps follower alive during waits so paths don't freeze.
    private void sleep(long millis) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < millis && !isStopRequested()) {
            follower.update();
            telemetry.update();
        }
    }

    // Simple flag to avoid spinning in sleep after stop (optional safety)
    private boolean isStopRequested() {
        // OpMode doesn't expose stopRequested(), but this is here
        // in case you later convert to LinearOpMode or add a flag.
        return false;
    }
}