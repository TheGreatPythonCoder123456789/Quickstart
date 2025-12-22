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
    private final Pose path4Pose = new Pose(84, 99);
    private final Pose path5Pose = new Pose();
    private final Pose path6Pose = new Pose();
    private final Pose path7Pose = new Pose();
    private final Pose path8Pose = new Pose();
    private final Pose path9Pose = new Pose();
    private final Pose path10Pose = new Pose();
    private final Pose path11Pose = new Pose();


    private PathChain path1, path2, path3;

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