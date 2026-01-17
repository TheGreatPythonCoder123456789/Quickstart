package org.firstinspires.ftc.teamcode.HP_Autonomous;



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous(name = "Pedro Shoot RED", group = "Autonomous")
public class hpred extends LinearOpMode {

    // ---------------- Core ----------------
    private Follower follower;
    private PathsRed paths;

    // ---------------- Hardware ----------------
    private DcMotor intakeTop;
    private ShooterSubsystem shooter;
    private Servo gate;

    // ---------------- State ----------------
    private enum AutoState {
        FOLLOW_PATH,
        SHOOT,
        DONE
    }

    private AutoState state = AutoState.FOLLOW_PATH;
    private AutoState lastState = null;

    // ---------------- Main ----------------
    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        paths = new PathsRed(follower);

        intakeTop = hardwareMap.get(DcMotor.class, "intakeTop");
        gate = hardwareMap.get(Servo.class, "gate");
        shooter = new ShooterSubsystem(hardwareMap);

        gate.setPosition(1.0);
        intakeTop.setPower(0);

        // RED START POSE (mirrored)
        follower.setPose(new Pose(50, 135, Math.toRadians(50)));

        waitForStart();

        while (opModeIsActive() && state != AutoState.DONE) {

            follower.update();

            switch (state) {

                case FOLLOW_PATH:
                    if (stateJustEntered()) {
                        follower.followPath(paths.Path1, false);
                    }
                    if (!follower.isBusy()) {
                        state = AutoState.SHOOT;
                    }
                    break;

                case SHOOT:
                    if (stateJustEntered()) {
                        follower.breakFollowing();
                        shootAllBalls();
                        state = AutoState.DONE;
                    }
                    break;

                case DONE:
                    break;
            }

            telemetry.addData("Auto", "RED");
            telemetry.addData("State", state);
            telemetry.update();
        }

        shooter.stopShooter();
        intakeTop.setPower(0);
    }

    // ---------------- State Helper ----------------
    private boolean stateJustEntered() {
        if (state != lastState) {
            lastState = state;
            return true;
        }
        return false;
    }

    // ---------------- Shooting ----------------
    private void shootAllBalls() throws InterruptedException {

        shooter.setTargetRPM(2200);
        waitForShooterReady();

        gate.setPosition(0.85);

        // Ball 1
        intakeTop.setPower(-1);
        sleep(300);
        intakeTop.setPower(0);
        sleep(500);

        // Ball 2
        intakeTop.setPower(-1);
        sleep(300);
        intakeTop.setPower(0);
        sleep(500);

        // Ball 3
        intakeTop.setPower(-1);
        sleep(500);

        intakeTop.setPower(0);
        gate.setPosition(1.0);
    }

    private void waitForShooterReady() {
        long start = System.currentTimeMillis();
        while (opModeIsActive()) {
            if (shooter.getLeftShooterVelocity() > 900 &&
                    shooter.getRightShooterVelocity() > 900) break;

            if (System.currentTimeMillis() - start > 1200) break;
            sleep(10);
        }
    }

    // ---------------- RED MIRRORED PATHS ----------------
    public static class PathsRed {

        public PathChain Path1;

        public PathsRed(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    // MIRRORED POSES
                                    new Pose(50, 135, Math.toRadians(50)),
                                    new Pose(67, 95),
                                    new Pose(75, 137, Math.toRadians(60))
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(50),
                            Math.toRadians(60)
                    )
                    .build();
        }
    }
}

