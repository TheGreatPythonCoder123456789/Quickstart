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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemCloseShooting;
import org.firstinspires.ftc.teamcode.OmarMingza.WorkingAprilTag.PIDControllerSubsystem;

@TeleOp(name="? pedro teleop 5", group="TeleOp")
public class pedroteleop5 extends LinearOpMode {

    // Hardware
    private DcMotor frontLeft, frontRight, backLeft, backRight, intakeTop;
    private Servo gate;
    private IMU imu;

    private ShooterSubsystemCloseShooting shooter;
    private HuskyLens huskyLens;

    // Pedro
    private Follower follower;
    private PathChain goToLaunchPath;

    // Pose
    private final Pose startPose = new Pose(0, 0, Math.toRadians(270));
    private final Pose launchingPose = new Pose(57.55, 89.84, Math.toRadians(-35));

    // April Tag constants
    private static final double CAMERA_FOV_X = 50.0;
    private static final int CAMERA_RES_X = 320;
    private static final double REAL_TAG_WIDTH = 4.0;

    private static final double CAMERA_X_OFFSET = 6.0;  // forward
    private static final double CAMERA_Y_OFFSET = 0.5;  // right

    private static final double TARGET_FORWARD = 36.0;
    private static final double TARGET_STRAFE = -5.0;
    private static final double TARGET_HEADING = 0.0;

    private static final double DIST_THRESHOLD = 5.0;
    private static final double STRAFE_THRESHOLD = 6.0;
    private static final double HEADING_THRESHOLD = 3.0;

    // PID controllers
    private PIDControllerSubsystem forwardPID;
    private PIDControllerSubsystem strafePID;
    private PIDControllerSubsystem headingPID;

    // State machine
    private enum AssistState {
        MANUAL,
        GO_TO_LAUNCH_POSE,
        WAIT_FOR_TAG,
        APRIL_ALIGN
    }
    private AssistState assistState = AssistState.MANUAL;

    // Headless
    private boolean headlessEnabled = true;
    private double botHeading = 0;

    // Button edge
    private boolean xPrev = false;

    // AprilTag timing
    private long tagSearchStartTime = 0;
    private static final long TAG_SEARCH_TIMEOUT_MS = 1500;

    @Override
    public void runOpMode() {

        initHardware();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        forwardPID = new PIDControllerSubsystem(0.02, 0, 0);
        strafePID  = new PIDControllerSubsystem(0.02, 0, 0);
        headingPID = new PIDControllerSubsystem(0.03, 0, 0);

        waitForStart();

        while (opModeIsActive()) {

            // Cancel button (B)
            if (gamepad1.b && assistState != AssistState.MANUAL) {
                follower.breakFollowing();
                assistState = AssistState.MANUAL;
                stopDrive();
            }

            // Press X to start Pedro → Launching Pose
            boolean xPressed = gamepad1.x && !xPrev;
            xPrev = gamepad1.x;

            if (xPressed && assistState == AssistState.MANUAL) {
                buildPathFromCurrentPose();
                follower.followPath(goToLaunchPath, false);
                assistState = AssistState.GO_TO_LAUNCH_POSE;
            }

            // State machine
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
                    HuskyLens.Block tag = findApril(5);

                    if (tag != null) {
                        resetAprilPID();
                        assistState = AssistState.APRIL_ALIGN;
                    }
                    else if (System.currentTimeMillis() - tagSearchStartTime > TAG_SEARCH_TIMEOUT_MS) {
                        assistState = AssistState.MANUAL;
                    }
                    else {
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

        if (gamepad2.right_trigger > 0) shooter.setTargetRPM(2050);
        else shooter.stopShooter();
    }

    private void stopDrive() {
        setDrivePower(0,0,0,0);
    }

    private void setDrivePower(double lf, double rf, double lb, double rb) {
        frontLeft.setPower(lf);
        frontRight.setPower(rf);
        backLeft.setPower(lb);
        backRight.setPower(rb);
    }

    private void buildPathFromCurrentPose() {
        Pose current = follower.getPose();

        goToLaunchPath = follower.pathBuilder()
                .addPath(new BezierLine(current, launchingPose))
                .setLinearHeadingInterpolation(current.getHeading(), launchingPose.getHeading())
                .setConstraints(new PathConstraints(
                        0.69,
                        9.2,
                        0.69,
                        0.69
                ))
                .build();
    }

    private void resetAprilPID() {
        forwardPID.reset();
        strafePID.reset();
        headingPID.reset();
    }

    private HuskyLens.Block findApril(int idd) {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        for (HuskyLens.Block b : blocks) {
            if (b.id == idd) return b;
        }
        return null;
    }

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

        if (forwardError == 0 && strafeError == 0 && headingError == 0) {
            stopDrive();
            assistState = AssistState.MANUAL;
            return;
        }

        double dt = 0.03;
        double[] powers = pidDrive(-forwardError, strafeError, headingError, dt);

        setDrivePower(
                powers[0] + powers[1] + powers[2],
                powers[0] - powers[1] - powers[2],
                powers[0] - powers[1] + powers[2],
                powers[0] + powers[1] - powers[2]
        );
    }

    private double[] pidDrive(double forwardError, double strafeError, double headingError, double dt) {
        double forwardPower = forwardPID.calculate(forwardError, dt);
        double strafePower  = strafePID.calculate(strafeError, dt);
        double turnPower    = headingPID.calculate(headingError, dt);

        forwardPower = Math.max(-1, Math.min(1, forwardPower));
        strafePower  = Math.max(-1, Math.min(1, strafePower));
        turnPower    = Math.max(-1, Math.min(1, turnPower));

        return new double[]{forwardPower, strafePower, turnPower};
    }
}
