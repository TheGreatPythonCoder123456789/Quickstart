package org.firstinspires.ftc.teamcode.Tests;

//for positioning robot make it on red tape by aligning it with
// the right and left ends of the C channels (end of the C channels)

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name="Far shooting test TELEOP", group="TeleOp")
public class teleopFarShootingTest extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intakeTop;
    private Servo gate;
    private IMU imu;

    // Shooter subsystem
    private ShooterSubsystem shooter;

    // Button state tracking
    boolean dpadUpPrevious = false;
    boolean yButtonPrevious = false;  // for headless toggle

    // Gate button edge detection
    boolean xPrev = false;
    boolean bPrev = false;

    // Speed mode button edge detection
    boolean lbPrev = false;
    boolean rbPrev = false;

    boolean headlessEnabled = true;
    boolean gateOpen = false;

    double botHeading = 0.0;
    double backNum = 90; //126 for wide (gate range for opening)

    // NEW: speed divisor (default slow mode)
    double speedDivisor = 1.8;

    // Intake slow mode toggle
    boolean intakeSlowMode = false;
    boolean dpadDownPrev = false;

    // ------------------ Adjustable Shooter RPM ------------------
    int shooterRPM = 3100;  // starting RPM
    boolean gp1DpadUpPrev = false;
    boolean gp1DpadDownPrev = false;

    @Override
    public void runOpMode() {
        initializeHardware();
        shooter = new ShooterSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
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
        gate = hardwareMap.get(Servo.class, "gate");
        imu = hardwareMap.get(IMU.class, "imu");

        intakeTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        double gateStartPos = gate.getPosition();   // read the servo’s actual physical position
        gate.setPosition(gateStartPos);      // treat that as the new “1.0”
    }

    // ---------------------------------------------------------
    //  COMBINED SHOOTING FUNCTION
    // ---------------------------------------------------------
    private void shootAllBalls() {

        // Spin up shooter
        shooter.setTargetRPM(2100);
        sleep(1500);

        // Ball 1
        intakeTop.setPower(-1.0);
        sleep(300);
        intakeTop.setPower(0.5);
        sleep(700);

        intakeTop.setPower(0);
        sleep(300);

        // Ball 2
        intakeTop.setPower(-1.0);
        sleep(500);
        intakeTop.setPower(0.5);
        sleep(700);

        intakeTop.setPower(0);
        sleep(300);

        // Ball 3
        intakeTop.setPower(-1.0);
        sleep(800);

        // Stop everything
        intakeTop.setPower(0);
        shooter.stopShooter();
    }

    private void servoSetter() {
        double currentPos = gate.getPosition();
        double degreesBack = backNum;
        double totalDegrees = 1800.0;

        double positionChange = degreesBack / totalDegrees;
        double newPos = currentPos - positionChange;

        newPos = Math.max(0.0, Math.min(1.0, newPos));
        gate.setPosition(newPos);
    }

    private void runTeleop() {

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

        // ------------------ DRIVETRAIN ------------------
        double drive  = -gamepad1.left_stick_y / speedDivisor;
        double strafe =  gamepad1.left_stick_x / speedDivisor;
        double turn   =  gamepad1.right_stick_x;

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (gamepad1.a) imu.resetYaw();

        // Toggle headless mode
        if (gamepad1.y && !yButtonPrevious) {
            headlessEnabled = !headlessEnabled;
        }
        yButtonPrevious = gamepad1.y;

        // Apply headless transform
        if (headlessEnabled) {
            double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);
            strafe = rotX;
            drive = rotY;
        }

        double frontLeftPower  = drive + strafe + turn / 2;
        double frontRightPower = drive - strafe - turn / 2;
        double backLeftPower   = drive - strafe + turn / 2;
        double backRightPower  = drive + strafe - turn / 2;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // ------------------ INTAKE SLOW MODE TOGGLE ------------------
        boolean dpadDownPressed = gamepad2.dpad_down && !dpadDownPrev;
        if (dpadDownPressed) {
            intakeSlowMode = !intakeSlowMode; // toggle slow mode
        }
        dpadDownPrev = gamepad2.dpad_down;

        // ------------------ INTAKE ------------------
        double intakePower = 0.0;
        if (gamepad2.left_bumper) {
            intakePower = intakeSlowMode ? -0.5 : -1.0; // Balls IN
        } else if (gamepad2.right_bumper) {
            intakePower = intakeSlowMode ? 0.5 : 1.0;   // Balls OUT
        }
        intakeTop.setPower(intakePower);

        // ------------------ SHOOTER RPM ADJUSTMENT (GAMEPAD 1) ------------------
        boolean gp1Up = gamepad1.dpad_up && !gp1DpadUpPrev;
        boolean gp1Down = gamepad1.dpad_down && !gp1DpadDownPrev;

        if (gp1Up) shooterRPM += 50;
        if (gp1Down) shooterRPM -= 50;

        shooterRPM = Math.max(500, Math.min(6000, shooterRPM));

        gp1DpadUpPrev = gamepad1.dpad_up;
        gp1DpadDownPrev = gamepad1.dpad_down;

        // ------------------ SHOOTER ------------------
        if (gamepad2.right_trigger > 0) {
            shooter.setTargetRPM(shooterRPM);
        } else {
            shooter.stopShooter();
        }

        // ------------------ GATE TOGGLE ------------------
        boolean xPressed = gamepad2.x && !xPrev;
        boolean bPressed = gamepad2.b && !bPrev;

        if (xPressed) {
            gate.setPosition(1.0);   // close
            gateOpen = false;
        }

        if (bPressed) {
            servoSetter();           // open
            gateOpen = true;
        }

        xPrev = gamepad2.x;
        bPrev = gamepad2.b;

        // ------------------ SHOOTING SEQUENCE ------------------
        if (gamepad2.dpad_up && !dpadUpPrevious) {
            shootAllBalls();
        }
        dpadUpPrevious = gamepad2.dpad_up;

        // ------------------ TELEMETRY ------------------
        telemetry.addData("Shooter Left Velocity", shooter.getLeftShooterVelocity());
        telemetry.addData("Shooter Right Velocity", shooter.getRightShooterVelocity());
        telemetry.addData("Shooter Target RPM", shooterRPM);
        telemetry.addData("Intake Power", intakeTop.getPower());
        telemetry.addData("Intake Mode", intakeSlowMode ? "SLOW" : "FULL");
        telemetry.addData("Gate Position", gateOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Gate Raw Position", gate.getPosition());
        telemetry.addData("IMU Heading (Radians)", botHeading);
        telemetry.addData("Headless Mode", headlessEnabled ? "ON" : "OFF");
        telemetry.addData("Drive Speed Mode", speedDivisor == 1.0 ? "FULL" : "SLOW");
    }
}