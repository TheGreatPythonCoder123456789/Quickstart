package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
// when preloading back up the last ball a little (test there to find distance)
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="testBlue", group="TeleOp")
public class TestBlue extends OpMode {

    private enum AutoState {
        DRIVE_PATH1,
        SHOOT_PREP,
        FIRE_BALLS,
        DRIVE_PATH2,
        DRIVE_PATH3,
        DONE
    }

    private Follower follower;
    private Timer timer;

    private DcMotor shootLeft, shootRight, intakeTop;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private static final double SHOOTER_POWER = 0.35;
    // Assuming negative power runs intake for both firing and pickup
    private static final double INTAKE_REVERSE_POWER = -0.8; // Negative power for intake

    private static final double SHOOTER_SPINUP_TIME = 3.0;
    private static final double INTAKE_PULSE_TIME = 0.4;
    private static final double INTAKE_PAUSE_TIME = 1.0;
    private static final int NUM_BALLS_TO_FIRE = 3;

    private final Pose startPose = new Pose(21, 122, Math.toRadians(-44));
    private final Pose path1Pose = new Pose(60, 99, Math.toRadians(-31));
    private final Pose path2Pose = new Pose(44, 84, Math.toRadians(180));
    private final Pose path3Pose = new Pose(15, 84, Math.toRadians(180));

    private PathChain path1, path2, path3;

    private AutoState currentState;

    private boolean pathStarted = false;

    // Firing sequence variables
    private int pulsesCompleted = 0;
    private boolean intakeActive = false;

    @Override
    public void init() {
        timer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        shootLeft = hardwareMap.get(DcMotor.class, "shootLeft");
        shootRight = hardwareMap.get(DcMotor.class, "shootRight");
        intakeTop = hardwareMap.get(DcMotor.class, "intakeTop");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        shootLeft.setDirection(DcMotor.Direction.FORWARD);
        shootRight.setDirection(DcMotor.Direction.FORWARD);
        intakeTop.setDirection(DcMotor.Direction.FORWARD);

        setShooterPower(0);
        intakeTop.setPower(0);
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

    private void setShooterPower(double p) {
        shootLeft.setPower(p);
        shootRight.setPower(-p); // you said this is correct
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
                    setShooterPower(SHOOTER_POWER);
                    timer.resetTimer();
                    currentState = AutoState.SHOOT_PREP;
                }
                break;

            case SHOOT_PREP:
                if (timer.getElapsedTimeSeconds() >= SHOOTER_SPINUP_TIME) {
                    timer.resetTimer();
                    pulsesCompleted = 0;
                    intakeActive = false;
                    intakeTop.setPower(0);
                    currentState = AutoState.FIRE_BALLS;
                }
                break;

            case FIRE_BALLS:
                setShooterPower(SHOOTER_POWER);

                if (pulsesCompleted < NUM_BALLS_TO_FIRE) {
                    double t = timer.getElapsedTimeSeconds();

                    // Define the timeline for each pulse and pause
                    // Ball 0: Pulse (0.4s), Pause (1.0s) -> End at 1.4s
                    // Ball 1: Pulse (1.4s), Pause (1.0s) -> End at 2.4s (1.4 + 1.0 = 2.4s)
                    // Ball 1: Shooter Acceleration Delay (1.0s) -> End at 3.4s (2.4 + 1.0 = 3.4s)
                    // Ball 2: Pulse (1.4s), Pause (1.0s) -> End at 5.8s (3.4 + 1.4 + 1.0 = 5.8s)
                    double timeForBall0 = INTAKE_PULSE_TIME + INTAKE_PAUSE_TIME; // 0.4 + 1.0 = 1.4s
                    double timeForBall1 = INTAKE_PULSE_TIME + INTAKE_PAUSE_TIME; // 0.4 + 1.0 = 1.4s
                    double shooterAccelTime = 1.0; // Additional 1 second for shooter acceleration?/

                    // Check if we have completed each specific pulse/pause cycle based on the timeline
                    if (pulsesCompleted == 0) {
                        // Check if Ball 0's pulse+pause cycle is complete (reached 1.4s)
                        if (t >= timeForBall0) {
                            pulsesCompleted = 1; // Move to Ball 1
                            intakeTop.setPower(0); // Ensure intake is off when moving to next pulse
                            intakeActive = false;
                        } else {
                            // Still in Ball 0's cycle
                            boolean inPulse0 = (t < INTAKE_PULSE_TIME); // Ball 0 pulse is 0.4s
                            if (inPulse0 && !intakeActive) {
                                intakeTop.setPower(INTAKE_REVERSE_POWER); // Use negative power
                                intakeActive = true;
                            } else if (!inPulse0 && intakeActive) {
                                intakeTop.setPower(0);
                                intakeActive = false;
                            }
                        }
                    } else if (pulsesCompleted == 1) {
                        // Check if Ball 1's pulse+pause+acceleration cycle is complete (reached 3.4s)
                        if (t >= (timeForBall0 + timeForBall1 + shooterAccelTime)) {
                            pulsesCompleted = 2; // Move to Ball 2
                            intakeTop.setPower(0); // Ensure intake is off when moving to next pulse
                            intakeActive = false;
                        } else {
                            // Still in Ball 1's cycle (starts after timeForBall0 = 1.4s)
                            double timeIntoBall1Cycle = t - timeForBall0; // Time elapsed since Ball 1 started

                            // During the first 1.4 seconds of Ball 1's cycle, run intake
                            if (timeIntoBall1Cycle < timeForBall1) {
                                boolean inPulse1 = (timeIntoBall1Cycle < INTAKE_PULSE_TIME); // Ball 1 pulse is 0.4s
                                if (inPulse1 && !intakeActive) {
                                    intakeTop.setPower(INTAKE_REVERSE_POWER); // Use negative power
                                    intakeActive = true;
                                } else if (!inPulse1 && intakeActive) {
                                    intakeTop.setPower(0);
                                    intakeActive = false;
                                }
                            }
                            // During the last 1.0 second (acceleration delay), keep shooter running but don't run intake
                            // This is handled by the overall state keeping shooter running
                        }
                    } else if (pulsesCompleted == 2) { // Ball 2
                        // Check if Ball 2's pulse+pause cycle is complete (reached 3.4 + 1.4 + 1.0 = 5.8s)
                        if (t >= (timeForBall0 + timeForBall1 + shooterAccelTime + timeForBall1)) {
                            pulsesCompleted = 3; // All balls fired
                            intakeTop.setPower(0); // Ensure intake is off before moving to next state
                            intakeActive = false;
                            // Transition will happen on next loop iteration because pulsesCompleted is now >= NUM_BALLS_TO_FIRE
                        } else {
                            // Still in Ball 2's cycle (starts after timeForBall0 + timeForBall1 + shooterAccelTime = 3.4s)
                            double timeIntoBall2Cycle = t - (timeForBall0 + timeForBall1 + shooterAccelTime); // Time elapsed since Ball 2 started
                            boolean inPulse2 = (timeIntoBall2Cycle < INTAKE_PULSE_TIME); // Ball 2 pulse is 0.4s
                            if (inPulse2 && !intakeActive) {
                                intakeTop.setPower(INTAKE_REVERSE_POWER); // Use negative power
                                intakeActive = true;
                            } else if (!inPulse2 && intakeActive) {
                                intakeTop.setPower(0);
                                intakeActive = false;
                            }
                        }
                    }

                } else {
                    // All balls fired (pulsesCompleted >= NUM_BALLS_TO_FIRE)
                    setShooterPower(0);
                    intakeTop.setPower(0);
                    pathStarted = false;
                    currentState = AutoState.DRIVE_PATH2;
                }
                break;

            case DRIVE_PATH2:
                if (!pathStarted) {
                    follower.followPath(path2, true);
                    pathStarted = true;
                    intakeTop.setPower(INTAKE_REVERSE_POWER); // Use negative power for pickup
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
                    intakeTop.setPower(INTAKE_REVERSE_POWER); // Use negative power for pickup
                }
                if (!follower.isBusy()) {
                    intakeTop.setPower(0);
                    currentState = AutoState.DONE;
                }
                break;

            case DONE:
                setShooterPower(0);
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
        telemetry.addData("Shooter L", shootLeft.getPower());
        telemetry.addData("Shooter R", shootRight.getPower());
        telemetry.addData("Pulses Completed", pulsesCompleted);
        telemetry.addData("Intake Active", intakeActive);
        telemetry.addData("Timer", timer.getElapsedTimeSeconds());
    }
}