package org.firstinspires.ftc.teamcode.pedroPathing.PEDRO_TELEOP;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "? pedro teleop 1", group = "TeleOp")
public class pedroteleop extends OpMode {

    /* ================= PEDRO ================= */
    private Follower follower;
    private TelemetryManager telemetryM;

    /* ================= APRILTAG ================= */
    private HuskyLens huskyLens;

    public static double CAMERA_FOV_X = 50.0;
    public static int CAMERA_RES_X = 320;
    public static double REAL_TAG_WIDTH = 4.0;

    public static double CAMERA_X_OFFSET = 6.0;
    public static double CAMERA_Y_OFFSET = 0.5;

    /* ================= FIELD ================= */
    public static Pose START_POSE =
            new Pose(0, 0, Math.toRadians(270));

    public static Pose LAUNCHING_POSE =
            new Pose( 58, 93, Math.toRadians(-38));

    // ⚠️ MEASURE THIS ON YOUR FIELD
    public static Pose TAG_5_POSE =
            new Pose(16, 132, Math.toRadians(142)); //-38

    /* ================= STATE ================= */
    private enum TeleOpState {
        DRIVER,
        GO_TO_LAUNCH,
        APRIL_ALIGN
    }

    private TeleOpState state = TeleOpState.DRIVER;

    /* ================= PATH ================= */
    private PathChain goToLaunchPath;

    /* ================= TUNING ================= */
    public static double POSE_BLEND = 0.20;
    public static double DIST_TOLERANCE = 2.0;
    public static double HEADING_TOLERANCE_DEG = 2.0;

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

        goToLaunchPath = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), LAUNCHING_POSE))
                .setLinearHeadingInterpolation(
                        follower.getPose().getHeading(),
                        LAUNCHING_POSE.getHeading()
                )
                .build();

        telemetryM.addLine("Pedro + AprilTag TeleOp Initialized");
        telemetryM.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        /* ========== DRIVER OVERRIDES ========== */
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
                    state = TeleOpState.GO_TO_LAUNCH;
                }
                break;

            case GO_TO_LAUNCH:
                if (!follower.isBusy()) {
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
        telemetryM.addData("State", state.toString());
        telemetryM.addData("Slow Mode", slowMode);
        telemetryM.addData("Busy", follower.isBusy());

        telemetryM.addData("X", String.format("%.2f", p.getX()));
        telemetryM.addData("Y", String.format("%.2f", p.getY()));
        telemetryM.addData("Heading (deg)",
                String.format("%.2f", Math.toDegrees(p.getHeading())));

        telemetryM.addData("Tag Heading (deg)",
                String.format("%.2f",
                        Math.toDegrees(TAG_5_POSE.getHeading())));


        telemetryM.update();
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

        double distance =
                (REAL_TAG_WIDTH * CAMERA_RES_X) /
                        (2.0 * tag.width *
                                Math.tan(Math.toRadians(CAMERA_FOV_X / 2.0)));

        double headingErrorDeg =
                (tag.x - CAMERA_RES_X / 2.0) *
                        CAMERA_FOV_X / CAMERA_RES_X;

        double headingErrorRad = Math.toRadians(headingErrorDeg);

        Pose aprilPose = aprilToPedroPose(distance, headingErrorRad);
        Pose current = follower.getPose();

        Pose blended = new Pose(
                lerp(current.getX(), aprilPose.getX(), POSE_BLEND),
                lerp(current.getY(), aprilPose.getY(), POSE_BLEND),
                lerpAngle(current.getHeading(), aprilPose.getHeading(), POSE_BLEND)
        );

        follower.setPose(blended);

        telemetryM.addData("Distance(in)",
                String.format("%.2f", distance));
        telemetryM.addData("Heading Err(deg)",
                String.format("%.2f", headingErrorDeg));
        telemetryM.addData("April X",
                String.format("%.2f", aprilPose.getX()));
        telemetryM.addData("April Y",
                String.format("%.2f", aprilPose.getY()));

        return Math.abs(distance - CAMERA_X_OFFSET) < DIST_TOLERANCE
                && Math.abs(headingErrorDeg) < HEADING_TOLERANCE_DEG;
    }

    private Pose aprilToPedroPose(double distance, double headingError) {
        double heading = TAG_5_POSE.getHeading() + headingError;

        double x =
                TAG_5_POSE.getX()
                        - distance * Math.cos(heading)
                        - CAMERA_X_OFFSET * Math.cos(heading);

        double y =
                TAG_5_POSE.getY()
                        - distance * Math.sin(heading)
                        - CAMERA_X_OFFSET * Math.sin(heading);

        return new Pose(x, y, heading);
    }

    private HuskyLens.Block findApril(int id) {
        for (HuskyLens.Block b : huskyLens.blocks()) {
            if (b.id == id) return b;
        }
        return null;
    }

    /* ================= HELPERS ================= */

    private double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private double lerpAngle(double a, double b, double t) {
        double diff = Math.atan2(Math.sin(b - a), Math.cos(b - a));
        return a + diff * t;
    }
}
