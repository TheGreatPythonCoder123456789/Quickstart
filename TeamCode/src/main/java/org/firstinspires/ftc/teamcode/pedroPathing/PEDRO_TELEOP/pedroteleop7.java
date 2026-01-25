package org.firstinspires.ftc.teamcode.pedroPathing.PEDRO_TELEOP;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

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

@TeleOp(name = "? pedro teleop 7 TEST", group = "TeleOp")
public class pedroteleop7 extends LinearOpMode {

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

    /* ================= WHITE TRIANGLE ================= */

    private static final Pose[] WHITE_TRIANGLE = new Pose[]{
            new Pose(15, 128, 0),
            new Pose(25, 144, 0),
            new Pose(84, 144, 0),
            new Pose(84, 83, 0),
            new Pose(72, 72, 0)
    };

    private static final double TRANSLATION_LOCK_RADIUS = 4.0;

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

    /* ================= STATE ================= */

    private enum AssistState {
        MANUAL,
        GO_TO_LAUNCH_POSE,
        WAIT_FOR_TAG,
        APRIL_ALIGN
    }

    private AssistState assistState = AssistState.MANUAL;

    /* ================= TELEMETRY RPM ================= */

    private static final double TICKS_PER_REV = 537.7;

    private long lastTime;
    private double lastFL, lastFR, lastBL, lastBR, lastIntake;

    double RPM_value = 2050;

    /* ================= HEADLESS ================= */

    private boolean headlessEnabled = true;
    private double botHeading = 0;
    private boolean xPrev = false;

    private long tagSearchStartTime = 0;
    private static final long TAG_SEARCH_TIMEOUT_MS = 1500;

    /* ================= OPMODE ================= */

    @Override
    public void runOpMode() {

        initHardware();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        forwardPID = new PIDControllerSubsystem(0.02, 0, 0);
        strafePID  = new PIDControllerSubsystem(0.02, 0, 0);
        headingPID = new PIDControllerSubsystem(0.03, 0, 0);

        lastTime = System.nanoTime();

        waitForStart();

        while (opModeIsActive()) {

            updateTelemetry();

            if (gamepad1.b && assistState != AssistState.MANUAL) {
                follower.breakFollowing();
                assistState = AssistState.MANUAL;
                stopDrive();
            }

            boolean xPressed = gamepad1.x && !xPrev;
            xPrev = gamepad1.x;

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
                    follower.update();
                    if (!follower.isBusy()) {
                        follower.breakFollowing();
                        stopDrive();
                        tagSearchStartTime = System.currentTimeMillis();
                        assistState = AssistState.WAIT_FOR_TAG;
                    }
                    break;

                case WAIT_FOR_TAG:
                    if (findApril(5) != null) {
                        resetAprilPID();
                        assistState = AssistState.APRIL_ALIGN;
                    } else if (System.currentTimeMillis() - tagSearchStartTime > TAG_SEARCH_TIMEOUT_MS) {
                        assistState = AssistState.MANUAL;
                    } else {
                        stopDrive();
                    }
                    break;

                case APRIL_ALIGN:
                    regulateAprilTag();
                    break;
            }

            telemetry.update();
        }
    }

    /* ================= TELEOP ================= */

    private void runManualTeleop() {

        double drive  = -gamepad1.left_stick_y / 1.8;
        double strafe =  gamepad1.left_stick_x / 1.8;
        double turn   =  gamepad1.right_stick_x;

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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

        if (gamepad2.right_trigger > 0) shooter.setTargetRPM(RPM_value);
        else shooter.stopShooter();
    }

    /* ================= APRIL TAG ================= */

    private void regulateAprilTag() {

        HuskyLens.Block tag = findApril(5);
        if (tag == null) {
            stopDrive();
            assistState = AssistState.MANUAL;
            return;
        }

        double distanceToTag =
                (REAL_TAG_WIDTH * CAMERA_RES_X) /
                        (2.0 * tag.width * Math.tan(Math.toRadians(CAMERA_FOV_X / 2.0)));

        double forwardError = distanceToTag - TARGET_FORWARD - CAMERA_X_OFFSET;

        double strafeError =
                ((tag.x - CAMERA_RES_X / 2.0) * CAMERA_FOV_X / CAMERA_RES_X);
        strafeError = -(TARGET_STRAFE + CAMERA_Y_OFFSET) - strafeError;

        double headingError =
                (tag.x - CAMERA_RES_X / 2.0) * CAMERA_FOV_X / CAMERA_RES_X;

        if (Math.abs(forwardError) < DIST_THRESHOLD) forwardError = 0;
        if (Math.abs(strafeError) < STRAFE_THRESHOLD) strafeError = 0;
        if (Math.abs(headingError) < HEADING_THRESHOLD) headingError = 0;

        Pose p = follower.getPose();

        if (nearLaunchPose() || !isInsideWhiteTriangle(p)) {
            forwardError = 0;
            strafeError = 0;
        }

        double[] powers = pidDrive(-forwardError, strafeError, headingError, 0.03);

        setDrivePower(
                powers[0] + powers[1] + powers[2],
                powers[0] - powers[1] - powers[2],
                powers[0] - powers[1] + powers[2],
                powers[0] + powers[1] - powers[2]
        );
    }

    /* ================= TELEMETRY ================= */

    private void updateTelemetry() {

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;

        double fl = frontLeft.getCurrentPosition();
        double fr = frontRight.getCurrentPosition();
        double bl = backLeft.getCurrentPosition();
        double br = backRight.getCurrentPosition();
        double in = intakeTop.getCurrentPosition();

        double flRPM = ((fl - lastFL) / TICKS_PER_REV) / dt * 60;
        double frRPM = ((fr - lastFR) / TICKS_PER_REV) / dt * 60;
        double blRPM = ((bl - lastBL) / TICKS_PER_REV) / dt * 60;
        double brRPM = ((br - lastBR) / TICKS_PER_REV) / dt * 60;
        double intakeRPM = ((in - lastIntake) / TICKS_PER_REV) / dt * 60;

        lastFL = fl;
        lastFR = fr;
        lastBL = bl;
        lastBR = br;
        lastIntake = in;
        lastTime = now;

        Pose p = follower.getPose();
        double distToLaunch = Math.hypot(
                p.getX() - launchingPose.getX(),
                p.getY() - launchingPose.getY()
        );

        HuskyLens.Block tag = findApril(5);

        telemetry.addLine("===== STATE =====");
        telemetry.addData("Assist State", assistState);
        telemetry.addData("Headless", headlessEnabled);

        telemetry.addLine("\n===== DRIVETRAIN RPM =====");
        telemetry.addData("FL", "%.1f", flRPM);
        telemetry.addData("FR", "%.1f", frRPM);
        telemetry.addData("BL", "%.1f", blRPM);
        telemetry.addData("BR", "%.1f", brRPM);

        telemetry.addLine("\n===== SHOOTER =====");
        telemetry.addLine("Shooter Target: " + RPM_value);
        telemetry.addData("Current RPM", shooter.getLeftShooterVelocity());

        telemetry.addLine("\n===== INTAKE =====");
        telemetry.addData("RPM", "%.1f", intakeRPM);

        telemetry.addLine("\n===== GATE =====");
        telemetry.addData("Position", gate.getPosition());
        telemetry.addData("Status", gate.getPosition() > 0.5 ? "OPEN" : "CLOSED");

        telemetry.addLine("\n===== POSE =====");
        telemetry.addData("X", "%.2f", p.getX());
        telemetry.addData("Y", "%.2f", p.getY());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(p.getHeading()));

        telemetry.addLine("\n===== LAUNCH POSE =====");
        telemetry.addData("X", launchingPose.getX());
        telemetry.addData("Y", launchingPose.getY());
        telemetry.addData("Heading", Math.toDegrees(launchingPose.getHeading()));
        telemetry.addData("Dist", "%.2f", distToLaunch);
        telemetry.addData("Near Launch", nearLaunchPose());

        telemetry.addLine("\n===== WHITE TRIANGLE =====");
        telemetry.addData("Inside", isInsideWhiteTriangle(p));

        telemetry.addLine("\n===== APRIL TAG =====");
        telemetry.addData("Detected", tag != null);
        if (tag != null) {
            telemetry.addData("Tag X", tag.x);
            telemetry.addData("Tag Width", tag.width);
        }
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

        intakeTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void buildPathFromCurrentPose() {
        Pose current = follower.getPose();
        goToLaunchPath = follower.pathBuilder()
                .addPath(new BezierLine(current, launchingPose))
                .setLinearHeadingInterpolation(current.getHeading(), launchingPose.getHeading())
                .setConstraints(new PathConstraints(0.69, 9.2, 0.69, 0.69))
                .build();
    }

    private boolean nearLaunchPose() {
        Pose p = follower.getPose();
        return Math.hypot(
                p.getX() - launchingPose.getX(),
                p.getY() - launchingPose.getY()
        ) < TRANSLATION_LOCK_RADIUS;
    }

    private boolean isInsideWhiteTriangle(Pose p) {
        boolean inside = false;
        for (int i = 0, j = WHITE_TRIANGLE.length - 1; i < WHITE_TRIANGLE.length; j = i++) {
            double xi = WHITE_TRIANGLE[i].getX();
            double yi = WHITE_TRIANGLE[i].getY();
            double xj = WHITE_TRIANGLE[j].getX();
            double yj = WHITE_TRIANGLE[j].getY();

            boolean intersect =
                    ((yi > p.getY()) != (yj > p.getY())) &&
                            (p.getX() < (xj - xi) * (p.getY() - yi) / (yj - yi) + xi);

            if (intersect) inside = !inside;
        }
        return inside;
    }

    private void resetAprilPID() {
        forwardPID.reset();
        strafePID.reset();
        headingPID.reset();
    }

    private HuskyLens.Block findApril(int id) {
        for (HuskyLens.Block b : huskyLens.blocks()) {
            if (b.id == id) return b;
        }
        return null;
    }

    private double[] pidDrive(double f, double s, double h, double dt) {
        return new double[]{
                clamp(forwardPID.calculate(f, dt)),
                clamp(strafePID.calculate(s, dt)),
                clamp(headingPID.calculate(h, dt))
        };
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
