package org.firstinspires.ftc.teamcode.OmarMingzhe.Teleop;
//for positioning robot make it on red tape by alligning it with
// the right and left ends of the C channels (end of the C channels)
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.subsystems.PIDControllerSubsystem;

import java.util.concurrent.TimeUnit;

@TeleOp(name="NEW ROBOT TELEOP", group="New Robot")
public class newRobotTeleop extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intake;
    private DcMotorEx shooter;
    private DcMotor rotation;
    private DcMotor getball;
    private CRServo servo1, servo2;
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
    boolean last=false, flag=false;

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
    int aprilTagNum;
    long lastTime;
    boolean gg=false;
    ElapsedTime runtime;

    private void initializeHardware() {
        // Map drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "leftfront");
        frontRight = hardwareMap.get(DcMotor.class, "rightfront");
        backLeft = hardwareMap.get(DcMotor.class, "leftback");
        backRight = hardwareMap.get(DcMotor.class, "rightback");
        getball = hardwareMap.get(DcMotor.class, "getball");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.MILLISECONDS);
        aprilTagNum=1;
        rateLimit.expire();

        // Map mechanism motors
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        rotation = hardwareMap.get(DcMotor.class, "rotation");
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");
        // Map IMU
        imu = hardwareMap.get(IMU.class, "imu");

        /*
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        */
//        intakeTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //shootLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //shootRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        rotation.setDirection(DcMotor.Direction.REVERSE);
        double kP=0.02,kD=0.00001;
//        double kP=0;
        forwardPID = new PIDControllerSubsystem(kP, 0.0, kD);
        strafePID  = new PIDControllerSubsystem(kP, 0.0, kD);
        headingPID = new PIDControllerSubsystem(0.02, 0.0, kD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // ShooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(0.17,0.0,0.0,12.2);
        runtime = new ElapsedTime();
    }
    private void shootMechLast() {

    }
    private void shootfrst2Balls() {

    }
    private void runTeleop() {

        double drive = -gamepad1.left_stick_y ;
        double strafe = gamepad1.left_stick_x ;
        double turn  = gamepad1.right_stick_x;

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (gamepad1.a) {
            imu.resetYaw();
        }
        if(gamepad1.x){
            flaggg=true;
            runtime.reset();
            lastTime=0;
            forwardPID.reset();
            strafePID.reset();
            headingPID.reset();
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

        double frontLeftPower  = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower   = drive - strafe + turn;
        double backRightPower  = drive + strafe - turn;

        frontLeft.setPower(frontLeftPower*0.6);
        frontRight.setPower(frontRightPower*0.6);
        backLeft.setPower(backLeftPower*0.6);
        backRight.setPower(backRightPower*0.6);

        if (gamepad2.left_bumper) {
            intake.setPower(-1.0);
            servo1.setPower(-1.0);
            servo2.setPower(-1.0);
        }
        else if (gamepad2.right_bumper) {
            intake.setPower(1.0);
            servo1.setPower(1.0);
            servo2.setPower(1.0);
        }
        else {
            intake.setPower(0);
            servo1.setPower(0);
            servo2.setPower(0);
        }
        if(gamepad2.y){
            if(!last) {
                // Ready();
                shooter.setVelocity(1700);
            }
            else{
//                    ShooterL.setPower(0);
                shooter.setVelocity(0);
            }
            last=!last;
        }
        if(gamepad2.x){
            getball.setPower(-0.5);
        }
        else{
            getball.setPower(0);
        }
        if(gamepad2.a){
            gg=!gg;
        }
//        if (gamepad2.dpad_up && !dpadUpPrevious) {
//            shootfrst2Balls();
//            shootMechLast();
//        }

//        dpadUpPrevious = gamepad2.dpad_up;
        // Telemetry
        if(gg&&checkapril(aprilTagNum)){

            rotation.setPower(0);
            if (flag == false) {
                runtime.reset();
                lastTime = 0;
                headingPID.reset();
            }
            regulate1();
            flag = true;
        }
        else{
            if(gamepad2.left_stick_x>0){
                rotation.setPower(0.3);
            }
            else if(gamepad2.left_stick_x<0){
                rotation.setPower(-0.3);
            }
            else{
                rotation.setPower(0);
            }
            flag=false;

        }
//        telemetry.addData("Shooter Left Side Power", shootLeft.getPower());
//        telemetry.addData("shootRight Right Side Power", shootRight.getPower());
//        telemetry.addData("intakeTop Power", intakeTop.getPower());
//        telemetry.addData("Front Left", frontLeft.getPower());
//        telemetry.addData("Front Right", frontRight.getPower());
//        telemetry.addData("Back Left", backLeft.getPower());
//        telemetry.addData("Back Right", backRight.getPower());
//        telemetry.addData("IMU Heading (Radians)", botHeading);
        telemetry.addData("Headless Mode", headlessEnabled ? "ON" : "OFF");
        telemetry.addData("AprilFollow Mode", gg ? "ON" : "OFF");
//        telemetry.addData("Y Button State", gamepad1.y); // Optional: Add this to see the button state in telemetry

        telemetry.addData("shooter:", shooter.getVelocity());
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
    //    public void regulate(){
//        HuskyLens.Block tag = null;
//        tag=findApril(5);
//
//        double drive = 0;
//        double strafe = 0;
//        double turn = 0;
//        double[] powers={0,0,0};
//
//        if (tag != null) {
//            // -------- DISTANCE ESTIMATION --------
//            double distanceToTag =
//                    (REAL_TAG_WIDTH * CAMERA_RES_X) /
//                            (2.0 * tag.width * Math.tan(Math.toRadians(CAMERA_FOV_X / 2.0)));
//
//            // -------- ERRORS --------
//            double forwardError =
//                    distanceToTag - TARGET_FORWARD - CAMERA_X_OFFSET;
//
//            double strafeError =
//                    (tag.x - CAMERA_RES_X / 2.0) * CAMERA_FOV_X / CAMERA_RES_X;
//            strafeError = -(TARGET_STRAFE + CAMERA_Y_OFFSET);
//
//            double headingError =
//                    (tag.x - CAMERA_RES_X / 2.0) * CAMERA_FOV_X / CAMERA_RES_X;
//
//            boolean headingGood = Math.abs(headingError) < HEADING_THRESHOLD;
//            boolean strafeGood  = Math.abs(strafeError)  < STRAFE_THRESHOLD;
//            boolean forwardGood = Math.abs(forwardError) < DIST_THRESHOLD;
//            if(forwardGood){
//                forwardError=0;
//            }
//            if(strafeGood){
//                strafeError=0;
//            }
//            if(headingGood){
//                headingError=0;
//            }
//            // -------- PRIORITY CONTROL --------
//
//            if(headingGood&&strafeGood&&forwardGood){
//                flaggg=false;
//            }
//
//            double now = runtime.seconds();
//            double dt = now - lastTime;
//            lastTime = now;
//            powers = pidDrive(-forwardError, strafeError, headingError, dt);
//
//            // -------- TELEMETRY --------
//            telemetry.addData("Tag ID", tag.id);
//            telemetry.addData("Distance (in)", "%.1f", distanceToTag);
//            telemetry.addData("Forward Error", "%.1f", forwardError);
//            telemetry.addData("Strafe Error", "%.1f", strafeError);
//            telemetry.addData("Heading Error (deg)", "%.1f", headingError);
//            telemetry.addData("Heading Error (deg)",headingGood);
//            telemetry.addData("Aligned",
//                    headingGood && strafeGood && forwardGood);
//
//        } else {
//            telemetry.addLine("Tag 5 NOT FOUND");
//        }
//        if(gamepad1.b){
//            flaggg=false;
//        }
//        drive=powers[0];
//        strafe=powers[1];
//        turn=powers[2];
//
//        // -------- MECANUM OUTPUT --------
//        frontLeft.setPower(drive + strafe + turn);
//        frontRight.setPower(drive - strafe - turn);
//        backLeft.setPower(drive - strafe + turn);
//        backRight.setPower(drive + strafe - turn);
//
//        telemetry.update();
//
//    }
    public void regulate1(){
        HuskyLens.Block tag = null;
        tag=findApril(aprilTagNum);

        double turn = 0;
        double[] powers={0,0,0};

        if (tag != null) {
            // -------- DISTANCE ESTIMATION --------
            double headingError = (tag.x - CAMERA_RES_X / 2.0) * CAMERA_FOV_X / CAMERA_RES_X;

            boolean headingGood = Math.abs(headingError) < HEADING_THRESHOLD;
            if(headingGood){
                runtime.reset();
                lastTime=0;
                headingPID.reset();
                return;
            }
            // -------- PRIORITY CONTROL --------

            long now = System.nanoTime();
            double dt = (now - lastTime) / 1e9;
            lastTime = now;
            powers = pidDrive(0, 0, headingError, dt);

            // -------- TELEMETRY --------
            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("Heading Error (deg)", "%.1f", headingError);
            telemetry.addData("Heading Error (deg)",headingGood);
            telemetry.addData("Aligned",
                    headingGood);

        } else {
            telemetry.addLine("Tag NOT FOUND");
        }
        turn=powers[2];

        // -------- MECANUM OUTPUT --------
        rotation.setPower(turn);

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