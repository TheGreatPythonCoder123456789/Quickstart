package org.firstinspires.ftc.teamcode.OmarMingzhe.Teleop;

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

@TeleOp(name = "<>teleopRedSetting<>", group = "TeleOp")
public class teleopRedSetting extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight, intakeTop;
    private Servo gate;
    private IMU imu;

    private ShooterSubsystemCloseShooting shooter;
    private HuskyLens huskyLens;

    private Follower follower;
    private PathChain goToLaunchPath;
    private double imuu;

    private final Pose startPose = new Pose(135, 36, Math.toRadians(270));
    private final Pose launchingPose =
            new Pose(86.45, 89.84, Math.toRadians(213.5));

    private static final double CAMERA_FOV_X = 50.0;
    private static final int CAMERA_RES_X = 320;
    private static final double REAL_TAG_WIDTH = 4.0;

    private static final double CAMERA_X_OFFSET = 6.0;
    private static final double CAMERA_Y_OFFSET = 0.5;

    private static final double TARGET_FORWARD = 36.0;
    private static final double TARGET_STRAFE = -5.0;

    private static final double DIST_THRESHOLD = 5.0;
    private static final double STRAFE_THRESHOLD = 6.0;
    private static final double HEADING_THRESHOLD = 2.5;

    private PIDControllerSubsystem forwardPID;
    private PIDControllerSubsystem strafePID;
    private PIDControllerSubsystem headingPID;

    private long lastPIDTime = 0;

    private long shooterAutoStartTime = 0;
    private boolean shooterAutoActive = false;
    private static final long SHOOTER_AUTO_DURATION = 5000;

    enum AssistState {
        MANUAL,
        GO_TO_LAUNCH_POSE,
        WAIT_FOR_TAG,
        REGULATE_IMU,
        REGULATE_IMU_INI,
        APRIL_ALIGN
    }

    private AssistState assistState = AssistState.MANUAL;

    private boolean headlessEnabled = true;
    private boolean xPrev = false;
    private boolean xPrev2 = false;
    private boolean bPrev = false;
    boolean ff=false;
    private long tagSearchStartTime = 0;
    private static final long TAG_SEARCH_TIMEOUT_MS = 300;

    boolean gateOpen = false;
    double backNum = 90;

    private boolean rtPrev = false;

    boolean lbPrev = false;
    boolean rbPrev = false;

    @Override
    public void runOpMode() {

        initHardware();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        forwardPID = new PIDControllerSubsystem(0.05, 0, 0.00001);
        strafePID  = new PIDControllerSubsystem(0.05, 0, 0.00001);
        headingPID = new PIDControllerSubsystem(0.03, 0, 0.00001);

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            // ===== RIGHT TRIGGER CALIBRATION RESET =====
            boolean rtPressed = gamepad1.right_trigger > 0.5 && !rtPrev;
            rtPrev = gamepad1.right_trigger > 0.5;

            if (rtPressed) {

                // Stop any assist/path following (NO movement)
                follower.breakFollowing();
                assistState = AssistState.MANUAL;
                stopDrive();

                // Reset IMU so current physical direction becomes 0
                imu.resetYaw();

                // Because robot is physically facing 270 on field,
                // we tell Pedro that this IMU zero corresponds to 270°
                Pose calibratedPose = new Pose(135, 9, Math.toRadians(270));

                // Reset BOTH current pose and internal reference
                follower.setPose(calibratedPose);

                // Sync heading target variable
                imuu = Math.toRadians(270);
                ff=false;
                // Reset PID controllers to avoid snap
                headingPID.reset();
                forwardPID.reset();
                strafePID.reset();
                lastPIDTime = System.nanoTime();
            }

            if (shooterAutoActive) {
                shooter.setTargetRPM(1550);
                if (System.currentTimeMillis() - shooterAutoStartTime > SHOOTER_AUTO_DURATION) {
                    shooter.stopShooter();
                    shooterAutoActive = false;
                }
            }

            if (gamepad1.b) {
                follower.breakFollowing();
                assistState = AssistState.MANUAL;
                stopDrive();
            }

            boolean xPressed = gamepad1.x && !xPrev;
            xPrev = gamepad1.x;

            if (xPressed && assistState == AssistState.MANUAL) {
                shooterAutoStartTime = System.currentTimeMillis();
                shooterAutoActive = true;

                if(ff) {
                    headingPID.reset();
                    lastPIDTime = System.nanoTime();
                    assistState=AssistState.REGULATE_IMU_INI;
                } else {
                    buildPathFromCurrentPose();
                    follower.followPath(goToLaunchPath, false);
                    assistState = AssistState.GO_TO_LAUNCH_POSE;
                }
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
                    if (findApril(2) != null) {
                        forwardPID.reset();
                        strafePID.reset();
                        headingPID.reset();
                        lastPIDTime = System.nanoTime();
                        assistState = AssistState.APRIL_ALIGN;
                    } else if (System.currentTimeMillis() - tagSearchStartTime > TAG_SEARCH_TIMEOUT_MS) {
                        headingPID.reset();
                        lastPIDTime = System.nanoTime();
                        assistState=AssistState.REGULATE_IMU;
                    }
                    break;

                case REGULATE_IMU:
                case REGULATE_IMU_INI:
                    regulateImu();
                    break;

                case APRIL_ALIGN:
                    regulateAprilTag();
                    break;
            }

            /* ================= FULL DEBUG TELEMETRY ================= */

            Pose p = follower.getPose();

            telemetry.addLine("========== SYSTEM ==========");
            telemetry.addData("Assist State", assistState);
            telemetry.addData("Shooter Auto Active", shooterAutoActive);
            telemetry.addData("ff flag", ff);
            telemetry.addData("Headless Mode", headlessEnabled);

            telemetry.addLine("\n========== POSE ==========");
            telemetry.addData("X", "%.2f", p.getX());
            telemetry.addData("Y", "%.2f", p.getY());
            telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(p.getHeading()));

            telemetry.addLine("\n========== IMU ==========");
            telemetry.addData("Raw Heading (deg)",
                    "%.2f",
                    Math.toDegrees(
                            imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
                    ));
            telemetry.addData("Target IMU (deg)", Math.toDegrees(imuu));

            telemetry.addLine("\n========== DRIVETRAIN ==========");
            telemetry.addData("Front Left", "%.2f", frontLeft.getPower());
            telemetry.addData("Front Right", "%.2f", frontRight.getPower());
            telemetry.addData("Back Left", "%.2f", backLeft.getPower());
            telemetry.addData("Back Right", "%.2f", backRight.getPower());

            telemetry.addLine("\n========== APRIL TAG ==========");
            HuskyLens.Block debugTag = findApril( (assistState == AssistState.APRIL_ALIGN || assistState == AssistState.WAIT_FOR_TAG)
                    ? 2   // Blue uses 1 — change to 2 for red if you want hardcoded
                    : 2);

            if (debugTag != null) {
                telemetry.addData("Tag ID", debugTag.id);
                telemetry.addData("Tag X", debugTag.x);
                telemetry.addData("Tag Width", debugTag.width);
            } else {
                telemetry.addLine("Tag: NOT DETECTED");
            }

            telemetry.addLine("\n========== SHOOTER ==========");
            telemetry.addData("Target RPM", 1550);
            telemetry.addData("Left Velocity", shooter.getLeftShooterVelocity());
            telemetry.addData("Right Velocity", shooter.getRightShooterVelocity());

            telemetry.addLine("\n========== INTAKE ==========");
            telemetry.addData("Intake Power", intakeTop.getPower());

            telemetry.addLine("\n========== GATE ==========");
            telemetry.addData("Gate Position", gate.getPosition());
            telemetry.addData("Gate Open Flag", gateOpen);
            telemetry.update();
        }
    }

    private void runManualTeleop() {
        double speedDivisor = 0;
        // ------------------ SPEED MODE TOGGLE ------------------
        boolean lbPressed = gamepad1.left_bumper && !lbPrev;
        boolean rbPressed = gamepad1.right_bumper && !rbPrev;

        if (rbPressed) {
            speedDivisor = 1.0;   // full speed
        }

        if (lbPressed) {
            speedDivisor = 1.8;   // slow mode
        }

        lbPrev = gamepad1.left_bumper;
        rbPrev = gamepad1.right_bumper;

        double drive  = -gamepad1.left_stick_y / speedDivisor; //1.8
        double strafe =  gamepad1.left_stick_x / speedDivisor; //1.8
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

        if (!shooterAutoActive) {
            if (gamepad2.right_trigger > 0) shooter.setTargetRPM(1550);
            else shooter.stopShooter();
        }

        boolean x2 = gamepad2.x && !xPrev2;
        boolean b2 = gamepad2.b && !bPrev;

        if (x2) {
            gate.setPosition(1.0);
            gateOpen = false;
        }

        if (b2) {
            servoSetter();
            gateOpen = true;
        }

        xPrev2 = gamepad2.x;
        bPrev = gamepad2.b;
    }

    private void regulateAprilTag() {
        HuskyLens.Block tag = findApril(2);
        if (tag == null) return;

        double distanceToTag =
                (REAL_TAG_WIDTH * CAMERA_RES_X) /
                        (2.0 * tag.width * Math.tan(Math.toRadians(CAMERA_FOV_X / 2.0)));

        double forwardError = distanceToTag - TARGET_FORWARD - CAMERA_X_OFFSET;
        double strafeError = -(TARGET_STRAFE + CAMERA_Y_OFFSET);
        double headingError =
                (tag.x - CAMERA_RES_X / 2.0) * CAMERA_FOV_X / CAMERA_RES_X;

        if (Math.abs(forwardError) < DIST_THRESHOLD) forwardError = 0;
        if (Math.abs(strafeError) < STRAFE_THRESHOLD) strafeError = 0;
        if (Math.abs(headingError) < HEADING_THRESHOLD) headingError = 0;

        if (forwardError == 0 && strafeError == 0 && headingError == 0) {
            stopDrive();
            assistState = AssistState.MANUAL;
            imuu = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            return;
        }

        long now = System.nanoTime();
        double dt = (now - lastPIDTime) / 1e9;
        lastPIDTime = now;

        double f = clamp(forwardPID.calculate(-forwardError, dt));
        double s = clamp(strafePID.calculate(strafeError, dt));
        double t = clamp(headingPID.calculate(headingError, dt));

        setDrivePower(f + s + t, f - s - t, f - s + t, f + s - t);
    }

    private void regulateImu(){
        double headingError = normalizeRadians(-imuu+
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        headingError=Math.toDegrees(headingError);

        if (Math.abs(headingError) < 5) headingError = 0;

        if (headingError == 0) {
            stopDrive();
            if(assistState==AssistState.REGULATE_IMU) assistState = AssistState.WAIT_FOR_TAG;
            else {
                buildPathFromCurrentPose();
                follower.followPath(goToLaunchPath, false);
                assistState = AssistState.GO_TO_LAUNCH_POSE;
            }
            return;
        }

        long now = System.nanoTime();
        double dt = (now - lastPIDTime) / 1e9;
        lastPIDTime = now;

        double t = clamp(headingPID.calculate(headingError, dt));
        setDrivePower(t,-t,t,-t);
    }

    private void buildPathFromCurrentPose() {
        Pose current = follower.getPose();
        if(!ff || Math.abs(Math.toDegrees(normalizeRadians(-imuu+
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))))<=50){
            goToLaunchPath = follower.pathBuilder()
                    .addPath(new BezierLine(current, launchingPose))
                    .setLinearHeadingInterpolation(current.getHeading(), launchingPose.getHeading())
                    .build();
            ff=true;
        } else {
            goToLaunchPath = follower.pathBuilder()
                    .addPath(new BezierLine(current, launchingPose))
                    .build();
        }
    }

    private void servoSetter() {
        double newPos = gate.getPosition() - (backNum / 1800.0);
        gate.setPosition(Math.max(0.0, Math.min(1.0, newPos)));
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

    public static double normalizeRadians(double angle) {
        angle %= (2 * Math.PI);
        if (angle > Math.PI) angle -= 2 * Math.PI;
        if (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}