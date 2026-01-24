package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "! AprilTagTest", group = "TeleOp")
public class NegroTags extends LinearOpMode {

    // ---------------- HARDWARE ----------------
    private DcMotor frontLeft, frontRight, backLeft, backRight;
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
    private static final double TARGET_HEADING = 0.0;  // deg

    // ---------------- THRESHOLDS ----------------
    private static final double DIST_THRESHOLD = 1.0;  // inches
    private static final double STRAFE_THRESHOLD = 1.0;
    private static final double HEADING_THRESHOLD = 3.0; // degrees

    // ---------------- CONTROL ----------------
    private static final double MOTOR_POWER = 0.3;

    @Override
    public void runOpMode() {

        // ---------------- MOTORS ----------------
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // ---------------- HUSKYLENS ----------------
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addLine("HuskyLens Bang-Bang Align Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            HuskyLens.Block[] blocks = huskyLens.blocks();
            HuskyLens.Block tag = null;

            for (HuskyLens.Block b : blocks) {
                if (b.id == 5) {
                    tag = b;
                    break;
                }
            }

            double drive = 0;
            double strafe = 0;
            double turn = 0;

            if (tag != null) {

                // -------- DISTANCE ESTIMATION --------
                double distanceToTag =
                        (REAL_TAG_WIDTH * CAMERA_RES_X) /
                                (2.0 * tag.width * Math.tan(Math.toRadians(CAMERA_FOV_X / 2.0)));

                // -------- ERRORS --------
                double forwardError =
                        distanceToTag - TARGET_FORWARD - CAMERA_X_OFFSET;

                double strafeError =
                        (tag.x - CAMERA_RES_X / 2.0) * CAMERA_FOV_X / CAMERA_RES_X;
                strafeError = -(TARGET_STRAFE + CAMERA_Y_OFFSET);

                double headingError =
                        (tag.x - CAMERA_RES_X / 2.0) * CAMERA_FOV_X / CAMERA_RES_X;

                boolean headingGood = Math.abs(headingError) < HEADING_THRESHOLD;
                boolean strafeGood  = Math.abs(strafeError)  < STRAFE_THRESHOLD;
                boolean forwardGood = Math.abs(forwardError) < DIST_THRESHOLD;

                // -------- PRIORITY CONTROL --------
                if (!headingGood) {
                    turn = headingError > 0 ? MOTOR_POWER : -MOTOR_POWER;
                } else if (!strafeGood) {
                    strafe = strafeError > 0 ? MOTOR_POWER : -MOTOR_POWER;
                } else if (!forwardGood) {
                    drive = forwardError > 0 ? MOTOR_POWER : -MOTOR_POWER;
                }

                // -------- TELEMETRY --------
                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Distance (in)", "%.1f", distanceToTag);
                telemetry.addData("Forward Error", "%.1f", forwardError);
                telemetry.addData("Strafe Error", "%.1f", strafeError);
                telemetry.addData("Heading Error (deg)", "%.1f", headingError);
                telemetry.addData("Aligned",
                        headingGood && strafeGood && forwardGood);

            } else {
                telemetry.addLine("Tag 5 NOT FOUND");
            }

            // -------- MECANUM OUTPUT --------
            frontLeft.setPower(drive + strafe + turn);
            frontRight.setPower(drive - strafe - turn);
            backLeft.setPower(drive - strafe + turn);
            backRight.setPower(drive + strafe - turn);

            telemetry.update();
        }
    }
}
