package org.firstinspires.ftc.teamcode.pedroPathing.PEDRO_TELEOP;
// add shooting controls
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
import org.firstinspires.ftc.teamcode.subsystems.PIDControllerSubsystem;

@TeleOp(name = "pedro teleop 10", group = "TeleOp")
public class pedroteleop10 extends LinearOpMode {

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

    /* ================= TOLERANCES (LOCAL) ================= */

    private double xTol, yTol, headingTol;

    private static final double PATH_X_TOL = 1.0;
    private static final double PATH_Y_TOL = 1.0;
    private static final double PATH_HEADING_TOL = Math.toRadians(2.0);

    private static final double MANUAL_X_TOL = 2.0;
    private static final double MANUAL_Y_TOL = 2.0;
    private static final double MANUAL_HEADING_TOL = Math.toRadians(6.0);

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
    private static final double HEADING_THRESHOLD = 3.0;

    /* ================= PID ================= */

    private PIDControllerSubsystem forwardPID;
    private PIDControllerSubsystem strafePID;
    private PIDControllerSubsystem headingPID;

    private long lastPIDTime = 0;

    /* ================= STATE ================= */

    private enum AssistState {
        MANUAL,
        GO_TO_LAUNCH_POSE,
        WAIT_FOR_TAG,
        APRIL_ALIGN
    }

    private AssistState assistState = AssistState.MANUAL;

    /* ================= INPUT ================= */

    private boolean headlessEnabled = true;
    private boolean xPrev = false;
    private boolean bPrev = false;

    private long tagSearchStartTime = 0;
    private static final long TAG_SEARCH_TIMEOUT_MS = 2000;

    /* ================= GATE ================= */

    private double backNum = 90;

    @Override
    public void runOpMode() {

        initHardware();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        setPoseTolerance(MANUAL_X_TOL, MANUAL_Y_TOL, MANUAL_HEADING_TOL);

        forwardPID = new PIDControllerSubsystem(0.03, 0, 0.00001);
        strafePID  = new PIDControllerSubsystem(0.03, 0, 0.00001);
        headingPID = new PIDControllerSubsystem(0.03, 0, 0.00001);

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            if (gamepad1.b) {
                follower.breakFollowing();
                setPoseTolerance(MANUAL_X_TOL, MANUAL_Y_TOL, MANUAL_HEADING_TOL);
                assistState = AssistState.MANUAL;
                stopDrive();
            }

            boolean xPressed = gamepad1.x && !xPrev;
            xPrev = gamepad1.x;

            if (xPressed && assistState == AssistState.MANUAL) {
                setPoseTolerance(PATH_X_TOL, PATH_Y_TOL, PATH_HEADING_TOL);
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
                        headingPID.reset();
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

            telemetry.addData("Assist", assistState);
            telemetry.update();
        }
    }

    /* ================= MANUAL ================= */

    private void runManualTeleop() {

        double drive  = -gamepad1.left_stick_y / 1.8;
        double strafe =  gamepad1.left_stick_x / 1.8;
        double turn   =  gamepad1.right_stick_x;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (gamepad1.a) imu.resetYaw();
        if (gamepad1.y) headlessEnabled = !headlessEnabled;

        if (headlessEnabled) {
            double rotX = strafe * Math.cos(-heading) - drive * Math.sin(-heading);
            double rotY = strafe * Math.sin(-heading) + drive * Math.cos(-heading);
            strafe = rotX;
            drive = rotY;
        }

        setDrivePower(
                drive + strafe + turn / 2,
                drive - strafe - turn / 2,
                drive - strafe + turn / 2,
                drive + strafe - turn / 2
        );
    }

    /* ================= APRIL TAG ================= */

    private void regulateAprilTag() {

        HuskyLens.Block tag = findApril(5);

        if (tag == null) {
            setDrivePower(0.12, -0.12, 0.12, -0.12);
            return;
        }

        double distance =
                (REAL_TAG_WIDTH * CAMERA_RES_X) /
                        (2.0 * tag.width * Math.tan(Math.toRadians(CAMERA_FOV_X / 2.0)));

        double forwardError = distance - TARGET_FORWARD - CAMERA_X_OFFSET;
        double headingError =
                (tag.x - CAMERA_RES_X / 2.0) * CAMERA_FOV_X / CAMERA_RES_X;

        if (Math.abs(forwardError) < DIST_THRESHOLD &&
                Math.abs(headingError) < HEADING_THRESHOLD) {

            stopDrive();
            assistState = AssistState.MANUAL;
            return;
        }

        long now = System.nanoTime();
        double dt = (now - lastPIDTime) / 1e9;
        lastPIDTime = now;

        double f = clamp(forwardPID.calculate(-forwardError, dt));
        double t = clamp(headingPID.calculate(headingError, dt));

        setDrivePower(
                f + t,
                f - t,
                f + t,
                f - t
        );
    }

    /* ================= HELPERS ================= */

    private void setPoseTolerance(double x, double y, double h) {
        xTol = x;
        yTol = y;
        headingTol = h;
    }

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
                .setLinearHeadingInterpolation(
                        current.getHeading(),
                        launchingPose.getHeading())
                .build();
    }

    private HuskyLens.Block findApril(int id) {
        for (HuskyLens.Block b : huskyLens.blocks()) {
            if (b.id == id) return b;
        }
        return null;
    }

    private double clamp(double v) {
        return Math.max(-1.0, Math.min(1.0, v));
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
