package org.firstinspires.ftc.teamcode.pedroPathing.PEDRO_TELEOP;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemCloseShooting;
import org.firstinspires.ftc.teamcode.OmarMingzaShazil.WorkingAprilTag.PIDControllerSubsystem;

@TeleOp(name = "pedro teleop 8 FINAL FIXED", group = "TeleOp")
public class pedroteleop8 extends LinearOpMode {

    /* ================= HARDWARE ================= */

    private DcMotor frontLeft, frontRight, backLeft, backRight, intakeTop;
    private Servo gate;
    private IMU imu;

    private ShooterSubsystemCloseShooting shooter;
    private HuskyLens huskyLens;

    /* ================= PEDRO ================= */

    private Follower follower;
    private PathChain goToLaunchPath;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(270));
    private final Pose launchingPose = new Pose(57.55, 89.84, Math.toRadians(-35));

    /* ================= APRIL TAG ================= */

    private static final double CAMERA_FOV_X = 50.0;
    private static final int CAMERA_RES_X = 320;
    private static final double REAL_TAG_WIDTH = 4.0;

    private static final double CAMERA_X_OFFSET = 6.0;
    private static final double CAMERA_Y_OFFSET = 0.5;

    private static final double TARGET_FORWARD = 36.0;
    private static final double TARGET_STRAFE = -5.0;

    private static final double DIST_THRESHOLD = 5.0;
    private static final double STRAFE_THRESHOLD = 6.0;

    /* ================= PID ================= */

    private PIDControllerSubsystem forwardPID;
    private PIDControllerSubsystem strafePID;

    private long lastPIDTime = 0;

    /* ================= STATE ================= */

    private enum AssistState {
        MANUAL,
        GO_TO_LAUNCH_POSE,
        WAIT_FOR_TAG,
        APRIL_ALIGN
    }

    private AssistState assistState = AssistState.MANUAL;

    /* ================= HEADLESS ================= */

    private boolean headlessEnabled = true;
    private boolean xPrev = false;

    private long tagSearchStartTime = 0;
    private static final long TAG_SEARCH_TIMEOUT_MS = 2000;

    /* ================= OPMODE ================= */

    @Override
    public void runOpMode() {

        initHardware();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        forwardPID = new PIDControllerSubsystem(0.02, 0, 0);
        strafePID  = new PIDControllerSubsystem(0.02, 0, 0);

        waitForStart();

        while (opModeIsActive()) {

            /* ===== CRITICAL FIX ===== */
            // Pedro pose MUST update every loop, even in manual
            follower.update();

            /* ===== CANCEL ASSIST ===== */
            if (gamepad1.b) {
                follower.breakFollowing();
                assistState = AssistState.MANUAL;
                stopDrive();
            }

            boolean xPressed = gamepad1.x && !xPrev;
            xPrev = gamepad1.x;

            /* ===== X BUTTON ===== */
            if (xPressed && assistState == AssistState.MANUAL) {
                buildPathFromCurrentPose();
                follower.followPath(goToLaunchPath, false);
                assistState = AssistState.GO_TO_LAUNCH_POSE;
            }

            switch (assistState) {

                case MANUAL:
                    runManualTeleop();
                    break;

                case GO_TO_LAUNCH_POSE:
                    if (!follower.isBusy()) {
                        follower.breakFollowing();
                        stopDrive();
                        tagSearchStartTime = System.currentTimeMillis();
                        assistState = AssistState.WAIT_FOR_TAG;
                    }
                    break;

                case WAIT_FOR_TAG:
                    if (findApril(5) != null) {
                        forwardPID.reset();
                        strafePID.reset();
                        lastPIDTime = System.nanoTime();
                        assistState = AssistState.APRIL_ALIGN;
                    } else if (System.currentTimeMillis() - tagSearchStartTime > TAG_SEARCH_TIMEOUT_MS) {
                        assistState = AssistState.MANUAL;
                    }
                    break;

                case APRIL_ALIGN:
                    regulateAprilTag();
                    break;
            }

            Pose p = follower.getPose();
            telemetry.addData("Assist State", assistState);
            telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f°",
                    p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
            telemetry.update();
        }
    }

    /* ================= MANUAL ================= */

    private void runManualTeleop() {

        double drive  = -gamepad1.left_stick_y / 1.8;
        double strafe =  gamepad1.left_stick_x / 1.8;
        double turn   =  gamepad1.right_stick_x;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (gamepad1.a) imu.resetYaw();
        if (gamepad1.y) headlessEnabled = !headlessEnabled;

        if (headlessEnabled) {
            double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);
            strafe = rotX;
            drive = rotY;
        }

        setDrivePower(
                drive + strafe + turn / 2,
                drive - strafe - turn / 2,
                drive - strafe + turn / 2,
                drive + strafe - turn / 2
        );

        if (gamepad2.left_bumper) intakeTop.setPower(-1);
        else if (gamepad2.right_bumper) intakeTop.setPower(1);
        else intakeTop.setPower(0);

        if (gamepad2.right_trigger > 0) shooter.setTargetRPM(2050);
        else shooter.stopShooter();
    }

    /* ================= APRIL TAG ALIGN ================= */

    private void regulateAprilTag() {

        HuskyLens.Block tag = findApril(5);

        if (tag == null) {
            setDrivePower(0.15, -0.15, 0.15, -0.15);
            return;
        }

        double distanceToTag =
                (REAL_TAG_WIDTH * CAMERA_RES_X) /
                        (2.0 * tag.width * Math.tan(Math.toRadians(CAMERA_FOV_X / 2.0)));

        double forwardError = distanceToTag - TARGET_FORWARD - CAMERA_X_OFFSET;

        double strafeError =
                ((tag.x - CAMERA_RES_X / 2.0) * CAMERA_FOV_X / CAMERA_RES_X);
        strafeError = -(TARGET_STRAFE + CAMERA_Y_OFFSET) - strafeError;

        if (Math.abs(forwardError) < DIST_THRESHOLD) forwardError = 0;
        if (Math.abs(strafeError) < STRAFE_THRESHOLD) strafeError = 0;

        if (forwardError == 0 && strafeError == 0) {
            stopDrive();
            return;
        }

        long now = System.nanoTime();
        double dt = (now - lastPIDTime) / 1e9;
        lastPIDTime = now;

        double f = clamp(forwardPID.calculate(-forwardError, dt));
        double s = clamp(strafePID.calculate(strafeError, dt));

        setDrivePower(
                f + s,
                f - s,
                f - s,
                f + s
        );
    }

    /* ================= HELPERS ================= */

    private void initHardware() {

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        intakeTop = hardwareMap.get(DcMotor.class, "intakeTop");
        gate = hardwareMap.get(Servo.class, "gate");
        imu = hardwareMap.get(IMU.class, "imu");

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        shooter = new ShooterSubsystemCloseShooting(hardwareMap);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    private void buildPathFromCurrentPose() {
        Pose current = follower.getPose();
        goToLaunchPath = follower.pathBuilder()
                .addPath(new BezierLine(current, launchingPose))
                .setLinearHeadingInterpolation(current.getHeading(), launchingPose.getHeading())
                .build();
    }

    private HuskyLens.Block findApril(int id) {
        for (HuskyLens.Block b : huskyLens.blocks()) {
            if (b.id == id) return b;
        }
        return null;
    }

    private double clamp(double v) {
        return Math.max(-1, Math.min(1, v));
    }

    private void stopDrive() {
        setDrivePower(0, 0, 0, 0);
    }

    private void setDrivePower(double lf, double rf, double lb, double rb) {
        frontLeft.setPower(lf);
        frontRight.setPower(rf);
        backLeft.setPower(lb);
        backRight.setPower(rb);
    }
}
