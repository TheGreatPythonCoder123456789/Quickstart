package org.firstinspires.ftc.teamcode;
//for positioning robot make it on red tape by alligning it with
// the right and left ends of the C channels (end of the C channels)
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="TeleopHeadlessAndroidStudio", group="TeleOp")
public class teleopHeadlessAndroidStudio extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intakeTop;
    private DcMotor shootLeft;
    private DcMotor shootRight;
    // IMU for headless mode
    private IMU imu;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            runTeleop();
            telemetry.update();
        }
    }

    boolean dpadUpPrevious = false; // Track state for gamepad2.dpad_up
    boolean yButtonPrevious = false; // Track state for gamepad1.y
    boolean dpadDownPrevious = false; // Track state for gamepad2.dpad_down (though not used for dpad_up toggle anymore, keeping for consistency if needed elsewhere)
    boolean headlessEnabled = true; // Toggle for headless mode
    double botHeading = 0.0; // Store current heading in radians

    private void initializeHardware() {
        // Map drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Map mechanism motors
        intakeTop = hardwareMap.get(DcMotor.class, "intakeTop");
        shootLeft = hardwareMap.get(DcMotor.class, "shootLeft");
        shootRight = hardwareMap.get(DcMotor.class, "shootRight");

        // Map IMU
        imu = hardwareMap.get(IMU.class, "imu");

        /*
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        */
        intakeTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //shootLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //shootRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }
    private void shootMechLast() {

    }
    private void shootfrst2Balls() {

    }
    private void runTeleop() {
        // Get joystick input
        double drive = -gamepad1.left_stick_y / 1.5; // Forward/backward
        double strafe = gamepad1.left_stick_x / 1.5; // strafe Left/right
        double turn  = gamepad1.right_stick_x; // Turn left/right

        //Headless mode:
        // Read current robot heading from IMU
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Button to reset IMU yaw (set current orientation as 'forward' for field-centric)
        if (gamepad1.a) {
            imu.resetYaw();
        }

        // Toggle headless mode with gamepad1.y button using its own state tracking
        if (gamepad1.y && !yButtonPrevious) {
            headlessEnabled = !headlessEnabled;
        }
        // Update the state of gamepad1.y for the next loop iteration
        yButtonPrevious = gamepad1.y;

        // If headless mode is enabled, perform field-centric calculations
        if (headlessEnabled) {
            // Rotate the joystick input coordinates to compensate for the robot's heading
            double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

            // Use the rotated coordinates instead of raw inputs
            strafe = rotX;
            drive = rotY;
        }

        // Calculate motor powers for mecanum drive (using potentially rotated inputs)
        double frontLeftPower  = drive + strafe + turn / 2;
        double frontRightPower = drive - strafe - turn / 2;
        double backLeftPower   = drive - strafe + turn / 2;
        double backRightPower  = drive + strafe - turn / 2;

        //

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        //strafing
        /*
        if(strafe <= 1 && strafe > 0) {
            frontLeft.setPower(1);
            backLeft.setPower(-1);
            frontRight.setPower(-1);
            backRight.setPower(1);
        } else if (strafe < 0 && strafe >= -1) {
            frontLeft.setPower(1);
            backLeft.setPower(-1);
            frontRight.setPower(-1);
            backRight.setPower(1);
        } else {
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }
        */

        // intakeTop Control
        if (gamepad2.left_bumper) {
            intakeTop.setPower(-1.0);
        } else if (gamepad2.right_bumper) {
            intakeTop.setPower(1.0);
        } else {
            intakeTop.setPower(0);
        }

        // Shooter Control
        if (gamepad2.right_trigger > 0) { //shoot out
            shootLeft.setPower(.35); //6000 RPM divided by 100
            shootRight.setPower(-.35);
        } else {
            shootLeft.setPower(0);
            shootRight.setPower(0);
        }
        /*
        if (gamepad2.left_trigger > 0) {
            shootLeft.setPower(-.4); //6000 RPM divided by 100
            shootRight.setPower(1.4); // shoot in
        } else {
            shootLeft.setPower(0);
            shootRight.setPower(0);
        }
        */
        if (gamepad2.dpad_up && !dpadUpPrevious) {
            shootfrst2Balls();
            shootMechLast();
        }

        // Update previous state for gamepad2.dpad_up
        dpadUpPrevious = gamepad2.dpad_up;

        //test control with all 4 wheels
        //qwerty power variable:
        /*
        double qwerty = 0.5;
        if (gamepad1.y) {
          backLeft.setPower(qwerty);
        } else {
          backLeft.setPower(0);
        }

        if (gamepad1.x) {
          frontLeft.setPower(qwerty);
        } else {
          frontLeft.setPower(0);
        }

        if (gamepad1.a) {
          backRight.setPower(qwerty);
        } else {
          backRight.setPower(0);
        }

        if (gamepad1.b) {
          frontRight.setPower(qwerty);
        } else {
          frontRight.setPower(0);
        }
        */
        // Telemetry
        telemetry.addData("Shooter Left Side Power", shootLeft.getPower());
        telemetry.addData("shootRight Right Side Power", shootRight.getPower());
        telemetry.addData("intakeTop Power", intakeTop.getPower());
        telemetry.addData("Front Left", frontLeft.getPower());
        telemetry.addData("Front Right", frontRight.getPower());
        telemetry.addData("Back Left", backLeft.getPower());
        telemetry.addData("Back Right", backRight.getPower());
        telemetry.addData("IMU Heading (Radians)", botHeading);
        telemetry.addData("Headless Mode", headlessEnabled ? "ON" : "OFF");
        telemetry.addData("Y Button State", gamepad1.y); // Optional: Add this to see the button state in telemetry
        telemetry.update();
    }
}