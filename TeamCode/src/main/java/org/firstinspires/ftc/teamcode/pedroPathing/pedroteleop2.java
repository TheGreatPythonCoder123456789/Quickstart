package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled

@Configurable
@TeleOp(name = "Pedro + AprilTag TeleOp (CORRECT 3-PHASE)")
public class pedroteleop2 extends OpMode {

    /* ================= PEDRO ================= */
    private Follower follower;
    private TelemetryManager telemetryM;

    /* ================= APRILTAG ================= */
    private HuskyLens huskyLens;

    public static double CAMERA_FOV_X = 50.0;
    public static int CAMERA_RES_X = 320;

    /* ================= FIELD ================= */

    // Robot starts BACKWARD because camera/shooter face backward
    public static Pose START_POSE =
            new Pose(0, 0, Math.toRadians(270));

    public static Pose LAUNCHING_POSE =
            new Pose(58, 93, 0); // heading ignored

    // Tag heading points OUT of goal
    public static Pose TAG_5_POSE =
            new Pose(16, 132, Math.toRadians(142));

    // Camera-facing field heading
    public static double TARGET_CAMERA_HEADING =
            TAG_5_POSE.getHeading();

    // Robot FRONT heading
    public static double TARGET_ROBOT_HEADING =
            TARGET_CAMERA_HEADING - Math.PI;

    /* ================= STATE ================= */
    private enum TeleOpState {
        DRIVER,
        GO_TO_LAUNCH_XY,
        ROTATE_TO_TARGET_HEADING,
        APRIL_ALIGN
    }

    private TeleOpState state = TeleOpState.DRIVER;

    /* ================= PATH ================= */
    private PathChain goToLaunchPath;

    /* ================= TUNING ================= */
    public static double ROTATE_KP = 1.2;

    public static double APRIL_YAW_GAIN = 0.4;
    public static double MAX_APRIL_YAW_DEG = 8.0;
    public static double APRIL_DONE_DEG = 1.5;

    public static double SLOW_MULT = 0.4;
    private boolean slowMode = false;

    /* ================================================= */

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        // X/Y ONLY — NO HEADING
        goToLaunchPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        follower.getPose(),
                        new Pose(LAUNCHING_POSE.getX(), LAUNCHING_POSE.getY())
                ))
                .build();

        telemetryM.addLine("Pedro + AprilTag 3-Phase Ready");
        telemetryM.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        /* ========== DRIVER OVERRIDE ========== */
        if (gamepad1.bWasPressed()) {
            follower.breakFollowing();
            follower.startTeleopDrive();
            state = TeleOpState.DRIVER;
        }

        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        /* ========== STATE MACHINE ========== */
        switch (state) {

            case DRIVER:
                double mult = slowMode ? SLOW_MULT : 1.0;

                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * mult,
                        -gamepad1.left_stick_x * mult,
                        -gamepad1.right_stick_x * mult,
                        true
                );

                if (gamepad1.xWasPressed()) {
                    follower.followPath(goToLaunchPath);
                    state = TeleOpState.GO_TO_LAUNCH_XY;
                }
                break;

            case GO_TO_LAUNCH_XY:
                if (!follower.isBusy()) {
                    state = TeleOpState.ROTATE_TO_TARGET_HEADING;
                }
                break;

            case ROTATE_TO_TARGET_HEADING:
                double headingError =
                        angleWrap(TARGET_ROBOT_HEADING -
                                follower.getPose().getHeading());

                follower.setTeleOpDrive(
                        0,
                        0,
                        headingError * ROTATE_KP,
                        true
                );

                if (Math.abs(Math.toDegrees(headingError)) < 2.0) {
                    follower.setTeleOpDrive(0, 0, 0, true);
                    state = TeleOpState.APRIL_ALIGN;
                }
                break;

            case APRIL_ALIGN:
                if (applyAprilCorrection()) {
                    follower.startTeleopDrive();
                    state = TeleOpState.DRIVER;
                }
                break;
        }

        /* ========== TELEMETRY ========== */
        Pose p = follower.getPose();

        telemetryM.addLine("===== PEDRO + APRILTAG =====");
        telemetryM.addData("State", state);

    }

    /* ================================================= */
    /* ================= APRILTAG ====================== */
    /* ================================================= */

    private boolean applyAprilCorrection() {
        HuskyLens.Block tag = findApril(5);

        telemetryM.addLine("----- APRILTAG -----");

        if (tag == null) {
            telemetryM.addLine("Tag 5: NOT FOUND");
            return false;
        }

        double yawErrorDeg =
                (tag.x - CAMERA_RES_X / 2.0) *
                        CAMERA_FOV_X / CAMERA_RES_X;

        double yawErrorRad = Math.toRadians(yawErrorDeg);

        yawErrorRad = Math.max(
                Math.min(yawErrorRad, Math.toRadians(MAX_APRIL_YAW_DEG)),
                Math.toRadians(-MAX_APRIL_YAW_DEG)
        );

        Pose current = follower.getPose();

        double newHeading =
                current.getHeading() + yawErrorRad * APRIL_YAW_GAIN;

        follower.setPose(new Pose(
                current.getX(),
                current.getY(),
                newHeading
        ));



        return Math.abs(yawErrorDeg) < APRIL_DONE_DEG;
    }

    private HuskyLens.Block findApril(int id) {
        for (HuskyLens.Block b : huskyLens.blocks()) {
            if (b.id == id) return b;
        }
        return null;
    }

    private double angleWrap(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
