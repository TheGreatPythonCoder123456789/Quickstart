package org.firstinspires.ftc.teamcode.pedroPathing;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.OmarMingza.WorkingAprilTag.PIDControllerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemCloseShooting;

@TeleOp(name="? pedro teleop 2", group="TeleOp")
public class pedroteleop2 extends LinearOpMode {

    // ================= HARDWARE =================
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeTop;
    private Servo gate;
    private IMU imu;

    private ShooterSubsystemCloseShooting shooter;

    // ================= PEDRO =================
    private Follower follower;
    private PathChain goToLaunchPath;

    private final Pose startPose =
            new Pose(0, 0, Math.toRadians(270));

    private final Pose launchingPose =
            new Pose(57.55, 89.84, Math.toRadians(-35));

    private static final PathConstraints TELEOP_CONSTRAINTS =
            new PathConstraints(0.6, 8.0, 0.5, 0.5);

    // ================= APRIL TAG =================
    private HuskyLens huskyLens;

    private PIDControllerSubsystem forwardPID;
    private PIDControllerSubsystem strafePID;
    private PIDControllerSubsystem headingPID;

    private static final double CAMERA_FOV_X = 50.0;
    private static final int CAMERA_RES_X = 320;
    private static final double REAL_TAG_WIDTH = 4.0;

    private static final double TARGET_FORWARD = 36.0;
    private static final double TARGET_STRAFE = 0.0;
    private static final double DIST_THRESHOLD = 4.0;
    private static final double STRAFE_THRESHOLD = 4.0;
    private static final double HEADING_THRESHOLD = 3.0;

    // ================= STATE =================
    private enum AssistState {
        MANUAL,
        GO_TO_LAUNCH_POSE,
        APRIL_ALIGN
    }

    private AssistState assistState = AssistState.MANUAL;
    private boolean xPrev = false;

    private final ElapsedTime pidTimer = new ElapsedTime();

    // ================= TELEOP CONTROL =================
    private boolean headlessEnabled = true;
    private boolean yPrev = false;
    private double botHeading = 0.0;

    // =================================================
    @Override
    public void runOpMode() {

        // -------- Hardware --------
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        intakeTop  = hardwareMap.get(DcMotor.class, "intakeTop");
        gate       = hardwareMap.get(Servo.class, "gate");
        imu        = hardwareMap.get(IMU.class, "imu");

        shooter = new ShooterSubsystemCloseShooting(hardwareMap);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        intakeTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // -------- Pedro --------
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        buildPedroPath();

        // -------- AprilTag --------
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        forwardPID = new PIDControllerSubsystem(0.02, 0.0, 0.0);
        strafePID  = new PIDControllerSubsystem(0.02, 0.0, 0.0);
        headingPID = new PIDControllerSubsystem(0.03, 0.0, 0.0);

        telemetry.addLine("READY");
        telemetry.update();

        waitForStart();
        pidTimer.reset();

        // ================= MAIN LOOP =================
        while (opModeIsActive()) {

            follower.update();

            boolean xPressed = gamepad1.x && !xPrev;
            xPrev = gamepad1.x;

            if (xPressed && assistState == AssistState.MANUAL) {
                assistState = AssistState.GO_TO_LAUNCH_POSE;
                follower.followPath(goToLaunchPath, false);
            }

            switch (assistState) {

                case MANUAL:
                    runManualTeleop();
                    break;

                case GO_TO_LAUNCH_POSE:
                    stopDrive();

                    if (poseReached(launchingPose)) {
                        follower.breakFollowing();
                        resetAprilPID();
                        assistState = AssistState.APRIL_ALIGN;
                    }
                    break;

                case APRIL_ALIGN:
                    regulateAprilTag();
                    break;
            }

            telemetry.addData("Assist State", assistState);
            telemetry.update();
        }
    }

    // =================================================
    // ================= MANUAL TELEOP =================
    private void runManualTeleop() {

        double drive  = -gamepad1.left_stick_y / 1.8;
        double strafe =  gamepad1.left_stick_x / 1.8;
        double turn   =  gamepad1.right_stick_x;

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (gamepad1.a) imu.resetYaw();

        if (gamepad1.y && !yPrev) {
            headlessEnabled = !headlessEnabled;
        }
        yPrev = gamepad1.y;

        if (headlessEnabled) {
            double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);
            strafe = rotX;
            drive = rotY;
        }

        setDrive(
                drive + strafe + turn / 2,
                drive - strafe - turn / 2,
                drive - strafe + turn / 2,
                drive + strafe - turn / 2
        );

        // Intake
        if (gamepad2.left_bumper) intakeTop.setPower(-1);
        else if (gamepad2.right_bumper) intakeTop.setPower(1);
        else intakeTop.setPower(0);

        // Shooter
        if (gamepad2.right_trigger > 0) shooter.setTargetRPM(2050);
        else shooter.stopShooter();
    }

    // =================================================
    // ================= APRIL TAG =================
    private void regulateAprilTag() {

        HuskyLens.Block tag = findApril(5);
        if (tag == null) {
            stopDrive();
            return;
        }

        double distance =
                (REAL_TAG_WIDTH * CAMERA_RES_X) /
                        (2.0 * tag.width *
                                Math.tan(Math.toRadians(CAMERA_FOV_X / 2.0)));

        double forwardError = distance - TARGET_FORWARD;
        double strafeError  = -(tag.x - CAMERA_RES_X / 2.0) * CAMERA_FOV_X / CAMERA_RES_X;
        double headingError = strafeError;

        boolean aligned =
                Math.abs(forwardError) < DIST_THRESHOLD &&
                        Math.abs(strafeError)  < STRAFE_THRESHOLD &&
                        Math.abs(headingError) < HEADING_THRESHOLD;

        if (aligned) {
            stopDrive();
            assistState = AssistState.MANUAL;
            return;
        }

        double dt = pidTimer.seconds();
        pidTimer.reset();

        double[] p = pidDrive(forwardError, strafeError, headingError, dt);

        setDrive(
                p[0] + p[1] + p[2],
                p[0] - p[1] - p[2],
                p[0] - p[1] + p[2],
                p[0] + p[1] - p[2]
        );
    }

    // =================================================
    // ================= HELPERS =================
    private void buildPedroPath() {
        goToLaunchPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, launchingPose))
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        launchingPose.getHeading())
                .setConstraints(TELEOP_CONSTRAINTS)
                .build();
    }

    private boolean poseReached(Pose target) {
        Pose cur = follower.getPose();
        return Math.hypot(cur.getX() - target.getX(),
                cur.getY() - target.getY()) < 3.5;
    }

    private void resetAprilPID() {
        forwardPID.reset();
        strafePID.reset();
        headingPID.reset();
        pidTimer.reset();
    }

    private double[] pidDrive(double f, double s, double h, double dt) {
        return new double[] {
                forwardPID.calculate(f, dt),
                strafePID.calculate(s, dt),
                headingPID.calculate(h, dt)
        };
    }

    private HuskyLens.Block findApril(int id) {
        for (HuskyLens.Block b : huskyLens.blocks()) {
            if (b.id == id) return b;
        }
        return null;
    }

    private void setDrive(double lf, double rf, double lb, double rb) {
        frontLeft.setPower(lf);
        frontRight.setPower(rf);
        backLeft.setPower(lb);
        backRight.setPower(rb);
    }

    private void stopDrive() {
        setDrive(0, 0, 0, 0);
    }
}
