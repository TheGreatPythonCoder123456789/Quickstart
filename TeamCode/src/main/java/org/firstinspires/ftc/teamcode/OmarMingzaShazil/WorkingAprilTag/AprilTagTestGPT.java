package org.firstinspires.ftc.teamcode.OmarMingzaShazil.WorkingAprilTag;
//for positioning robot make it on red tape by alligning it with
// the right and left ends of the C channels (end of the C channels)
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name="WORKING APRIL TAG", group="Linear Opmode")
public class AprilTagTestGPT extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intakeTop;
    private DcMotor shootLeft;
    private DcMotor shootRight;
    private Servo gate;
    // IMU for headless mode
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
    private static final double TARGET_HEADING = 0.0;  // deg

    // ---------------- THRESHOLDS ----------------
    private static final double DIST_THRESHOLD = 5.0;  // inches
    private static final double STRAFE_THRESHOLD = 6.0;
    private static final double HEADING_THRESHOLD = 3.0; // degrees

    // ---------------- CONTROL ----------------
    private static final double MOTOR_POWER = 0.2;
    private final int READ_PERIOD = 30;
    public boolean flaggg=false;

    public PIDControllerSubsystem forwardPID;
    public PIDControllerSubsystem strafePID;
    public PIDControllerSubsystem headingPID;


    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            if(flaggg){
                regulate();
                continue;
            }
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

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.MILLISECONDS);

        rateLimit.expire();

        // Map mechanism motors
        intakeTop = hardwareMap.get(DcMotor.class, "intakeTop");
        shootLeft = hardwareMap.get(DcMotor.class, "shootLeft");
        shootRight = hardwareMap.get(DcMotor.class, "shootRight");
        gate = hardwareMap.get(Servo.class, "gate");

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
        double kP=0.02;
        forwardPID = new PIDControllerSubsystem(kP, 0.0, 0);
        strafePID  = new PIDControllerSubsystem(kP, 0.0, 0);
        headingPID = new PIDControllerSubsystem(0.03, 0.0, 0);
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
        if(gamepad1.x){
            flaggg=true;

            forwardPID.reset();
            strafePID.reset();
            headingPID.reset();
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
        if(checkapril(5)){

            telemetry.addLine("FOUND APRIL TAG");

        }
        else{

            telemetry.addLine("NONONONONNONO TAG");
        }
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
    public boolean checkapril(int idd){
        if(findApril(idd)==null)return false;
        return true;
    }
    public HuskyLens.Block findApril(int idd){
        HuskyLens.Block[] blocks = huskyLens.blocks();
        for (HuskyLens.Block b : blocks) {
            if (b.id == idd) {
                return b;
            }
        }
        return null;
    }
    public void regulate(){
        HuskyLens.Block tag = null;
        tag=findApril(5);

        double drive = 0;
        double strafe = 0;
        double turn = 0;
        double lastTime=0;
        double[] powers={0,0,0};

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

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
            if(forwardGood){
                forwardError=0;
            }
            if(strafeGood){
                strafeError=0;
            }
            if(headingGood){
                headingError=0;
            }
            // -------- PRIORITY CONTROL --------

            if(headingGood&&strafeGood&&forwardGood){
                flaggg=false;
            }

            double now = runtime.seconds();
            double dt = now - lastTime;
            lastTime = now;
            powers = pidDrive(-forwardError, strafeError, headingError, dt);

            // -------- TELEMETRY --------
            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("Distance (in)", "%.1f", distanceToTag);
            telemetry.addData("Forward Error", "%.1f", forwardError);
            telemetry.addData("Strafe Error", "%.1f", strafeError);
            telemetry.addData("Heading Error (deg)", "%.1f", headingError);
            telemetry.addData("Heading Error (deg)",headingGood);
            telemetry.addData("Aligned",
                    headingGood && strafeGood && forwardGood);

        } else {
            telemetry.addLine("Tag 5 NOT FOUND");
        }
        if(gamepad1.x){
            flaggg=false;
        }
        drive=powers[0];
        strafe=powers[1];
        turn=powers[2];

        // -------- MECANUM OUTPUT --------
        frontLeft.setPower(drive + strafe + turn);
        frontRight.setPower(drive - strafe - turn);
        backLeft.setPower(drive - strafe + turn);
        backRight.setPower(drive + strafe - turn);

        telemetry.update();

    }
    public double[] pidDrive(double forwardError, double strafeError, double headingError, double dt) {

        double forwardPower = forwardPID.calculate(forwardError, dt);
        double strafePower  = strafePID.calculate(strafeError, dt);
        double turnPower    = headingPID.calculate(headingError, dt);

        forwardPower = Math.max(-1, Math.min(1, forwardPower));
        strafePower  = Math.max(-1, Math.min(1, strafePower));
        turnPower    = Math.max(-1, Math.min(1, turnPower));

        return new double[]{forwardPower, strafePower, turnPower};
    }
}




