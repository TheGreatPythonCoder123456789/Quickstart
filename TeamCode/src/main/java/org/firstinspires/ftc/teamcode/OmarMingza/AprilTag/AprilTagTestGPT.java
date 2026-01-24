package org.firstinspires.ftc.teamcode.OmarMingza.AprilTag;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Test 1/23", group="collios")
public class AprilTagTestGPT extends LinearOpMode {

    // ---------------- DRIVE ----------------
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeTop, shootLeft, shootRight;
    private Servo gate;

    // ---------------- SENSORS ----------------
    private IMU imu;
    private HuskyLens huskyLens;

    // ---------------- CAMERA CONFIG ----------------
    private static final double CAMERA_FOV_X = 50.0;   // degrees
    private static final int CAMERA_RES_X = 320;
    private static final double REAL_TAG_WIDTH = 4.0;  // inches

    // Camera offset from robot center
    private static final double CAMERA_X_OFFSET = 6.0; // forward
    private static final double CAMERA_Y_OFFSET = 0.5; // right

    // ---------------- TARGETS ----------------
    private static final double TARGET_FORWARD = 36.0; // inches
    private static final double TARGET_STRAFE = 5.0;   // inches right
    private static final double TARGET_HEADING = 0.0;  // degrees

    // ---------------- THRESHOLDS ----------------
    private static final double DIST_THRESHOLD = 5.0;
    private static final double STRAFE_THRESHOLD = 6.0;
    private static final double HEADING_THRESHOLD = 3.0;

    // ---------------- CONTROL ----------------
    private static final double MOTOR_POWER = 0.2;

    // ---------------- STATE ----------------
    private boolean headlessEnabled = true;
    private boolean yButtonPrevious = false;
    private double botHeading = 0.0;

    @Override
    public void runOpMode() {

        initializeHardware();

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            runTeleop();
            telemetry.update();
        }
    }

    private void initializeHardware() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        intakeTop = hardwareMap.get(DcMotor.class, "intakeTop");
        shootLeft = hardwareMap.get(DcMotor.class, "shootLeft");
        shootRight = hardwareMap.get(DcMotor.class, "shootRight");
        gate = hardwareMap.get(Servo.class, "gate");

        imu = hardwareMap.get(IMU.class, "imu");

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    private void runTeleop() {

        // ---------------- DRIVER INPUT ----------------
        double drive = -gamepad1.left_stick_y / 1.5;
        double strafe = gamepad1.left_stick_x / 1.5;
        double turn = gamepad1.right_stick_x / 2.0;

        // ---------------- IMU ----------------
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (gamepad1.a) {
            imu.resetYaw();
        }

        if (gamepad1.y && !yButtonPrevious) {
            headlessEnabled = !headlessEnabled;
        }
        yButtonPrevious = gamepad1.y;

        if (headlessEnabled) {
            double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);
            strafe = rotX;
            drive = rotY;
        }



        // ---------------- APRILTAG ALIGN (HOLD X) ----------------
        if (gamepad1.x) {
            regulate();
            return; // prevent manual overwrite
        }

        // ---------------- MECANUM DRIVE ----------------
        frontLeft.setPower(drive + strafe + turn);
        frontRight.setPower(drive - strafe - turn);
        backLeft.setPower(drive - strafe + turn);
        backRight.setPower(drive + strafe - turn);

        // ---------------- INTAKE ----------------
        if (gamepad2.left_bumper) intakeTop.setPower(-1);
        else if (gamepad2.right_bumper) intakeTop.setPower(1);
        else intakeTop.setPower(0);

        // ---------------- SHOOTER ----------------
        if (gamepad2.right_trigger > 0) {
            shootLeft.setPower(0.35);
            shootRight.setPower(-0.35);
        } else {
            shootLeft.setPower(0);
            shootRight.setPower(0);
        }

        telemetry.addData("Headless", headlessEnabled);
        telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
    }

    // ================= APRILTAG ALIGN (ONE PASS) =================

    // Returns the AprilTag block with the given ID, or null if not found
    public HuskyLens.Block findApril(int idd) {
        if (huskyLens == null) return null; // safety check
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if (blocks == null) return null; // sometimes HuskyLens returns null
        for (HuskyLens.Block b : blocks) {
            if (b.id == idd) {
                return b;
            }
        }
        return null;
    }

    // Returns true if the AprilTag with the given ID is visible
    public boolean checkapril(int idd) {
        return findApril(idd) != null;
    }

    public void regulate() {

        int lostCount = 0;

        while (opModeIsActive()) {

            HuskyLens.Block tag = findApril(5);

            double drive = 0;
            double strafe = 0;
            double turn = 0;

            if (tag != null) {

                lostCount = 0;

                // -------- DISTANCE --------
                double distanceToTag =
                        (REAL_TAG_WIDTH * CAMERA_RES_X) /
                                (2.0 * tag.width * Math.tan(Math.toRadians(CAMERA_FOV_X / 2.0)));

                double forwardError =
                        distanceToTag - TARGET_FORWARD - CAMERA_X_OFFSET;

                // -------- HEADING --------
                double angleX =
                        (tag.x - CAMERA_RES_X / 2.0) * CAMERA_FOV_X / CAMERA_RES_X;

                double headingError =
                        angleX - TARGET_HEADING;

                // -------- STRAFE (FIXED) --------
                double strafeError =
                        angleX - TARGET_STRAFE - CAMERA_Y_OFFSET;

                boolean headingGood = Math.abs(headingError) < HEADING_THRESHOLD;
                boolean strafeGood = Math.abs(strafeError) < STRAFE_THRESHOLD;
                boolean forwardGood = Math.abs(forwardError) < DIST_THRESHOLD;

                // -------- PRIORITY --------
                if (!headingGood) {
                    turn = headingError > 0 ? MOTOR_POWER : -MOTOR_POWER;
                } else if (!strafeGood) {
                    strafe = strafeError > 0 ? MOTOR_POWER : -MOTOR_POWER;
                } else if (!forwardGood) {
                    drive = forwardError > 0 ? MOTOR_POWER : -MOTOR_POWER;
                } else {
                    break; // fully aligned
                }

                telemetry.addLine("ALIGNING");
                telemetry.addData("Forward Err", forwardError);
                telemetry.addData("Strafe Err", strafeError);
                telemetry.addData("Heading Err", headingError);

            } else {
                lostCount++;

                // STOP MOVEMENT WHEN TAG IS LOST
                drive = 0;
                strafe = 0;
                turn = 0;

                telemetry.addLine("TAG LOST");

                // If tag lost for too long → abort
                if (lostCount > 15) {
                    break;
                }
            }

            // -------- APPLY POWER --------
            frontLeft.setPower(drive + strafe + turn);
            frontRight.setPower(drive - strafe - turn);
            backLeft.setPower(drive - strafe + turn);
            backRight.setPower(drive + strafe - turn);

            telemetry.update();
        }

        // -------- HARD STOP --------
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}