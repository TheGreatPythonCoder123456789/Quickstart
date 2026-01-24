package org.firstinspires.ftc.teamcode.CompetitionYesWeight;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemCloseShooting;

@TeleOp(name="TeleopMeet4", group="TeleOp")
public class TeleopMeet4 extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeTop;
    private Servo gate;
    private IMU imu;

    private ShooterSubsystemCloseShooting shooter;

    // Button state tracking
    boolean dpadUpPrevious = false;
    boolean yButtonPrevious = false;
    boolean xPrev = false;
    boolean bPrev = false;
    boolean lbPrev = false;
    boolean rbPrev = false;
    boolean dpadDownPrev = false;

    // NEW: shooter PID mode toggle (close/far)
    boolean shooterFarMode = false;
    boolean dpadLeftPrev = false;

    boolean headlessEnabled = true;
    boolean gateOpen = false;

    double botHeading = 0.0;
    double backNum = 90;
    double speedDivisor = 1.8;
    boolean intakeSlowMode = false;

    @Override
    public void runOpMode() {

        initializeHardware();
        shooter = new ShooterSubsystemCloseShooting(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            runTeleop();
            telemetry.update();
        }
    }

    private void initializeHardware() {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        intakeTop  = hardwareMap.get(DcMotor.class, "intakeTop");
        gate       = hardwareMap.get(Servo.class, "gate");
        imu        = hardwareMap.get(IMU.class, "imu");

        intakeTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        double gateStartPos = gate.getPosition();
        gate.setPosition(gateStartPos);
    }

    // Combined shooting sequence
    private void shootAllBalls() {
        shooter.setTargetRPM(2050);
        sleep(1500);

        intakeTop.setPower(-1.0);
        sleep(300);
        intakeTop.setPower(0.5);
        sleep(700);
        intakeTop.setPower(0);
        sleep(300);

        intakeTop.setPower(-1.0);
        sleep(500);
        intakeTop.setPower(0.5);
        sleep(700);
        intakeTop.setPower(0);
        sleep(300);

        intakeTop.setPower(-1.0);
        sleep(800);

        intakeTop.setPower(0);
        shooter.stopShooter();
    }

    private void servoSetter() {
        double currentPos = gate.getPosition();
        double positionChange = backNum / 1800.0;
        double newPos = Math.max(0.0, Math.min(1.0, currentPos - positionChange));
        gate.setPosition(newPos);
    }

    private void runTeleop() {

        // ---------------- SPEED MODE ----------------
        boolean lbPressed = gamepad1.left_bumper && !lbPrev;
        boolean rbPressed = gamepad1.right_bumper && !rbPrev;

        if (rbPressed) speedDivisor = 1.0;
        if (lbPressed) speedDivisor = 1.8;

        lbPrev = gamepad1.left_bumper;
        rbPrev = gamepad1.right_bumper;

        // ---------------- DRIVETRAIN ----------------
        double drive = -gamepad1.left_stick_y / speedDivisor;
        double strafe = gamepad1.left_stick_x / speedDivisor;
        double turn = gamepad1.right_stick_x;

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (gamepad1.a) imu.resetYaw();

        if (gamepad1.y && !yButtonPrevious) headlessEnabled = !headlessEnabled;
        yButtonPrevious = gamepad1.y;

        if (headlessEnabled) {
            double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);
            strafe = rotX;
            drive = rotY;
        }

        double frontLeftPower = drive + strafe + turn / 2;
        double frontRightPower = drive - strafe - turn / 2;
        double backLeftPower = drive - strafe + turn / 2;
        double backRightPower = drive + strafe - turn / 2;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // ---------------- INTAKE SLOW MODE ----------------
        boolean dpadDownPressed = gamepad2.dpad_down && !dpadDownPrev;
        if (dpadDownPressed) intakeSlowMode = !intakeSlowMode;
        dpadDownPrev = gamepad2.dpad_down;

        // ---------------- INTAKE ----------------
        double intakePower = 0.0;

        if (gamepad2.left_bumper) {
            intakePower = intakeSlowMode ? -0.5 : -1.0;
        } else if (gamepad2.right_bumper) {
            intakePower = intakeSlowMode ? 0.5 : 1.0;
        }

        intakeTop.setPower(intakePower);

        // ---------------- SHOOTER PID MODE TOGGLE (D-PAD LEFT) ----------------
        boolean dpadLeftPressed = gamepad2.dpad_left && !dpadLeftPrev;

        if (dpadLeftPressed) {
            shooterFarMode = !shooterFarMode;

            if (shooterFarMode) {
                shooter.useFarPID();
            } else {
                shooter.useClosePID();
            }
        }

        dpadLeftPrev = gamepad2.dpad_left;

        // ---------------- SHOOTER RPM ----------------
        if (gamepad2.right_trigger > 0) {
            shooter.setTargetRPM(2050); // SAME RPM for both modes
        } else {
            shooter.stopShooter();
        }

        // ---------------- GATE ----------------
        boolean xPressed = gamepad2.x && !xPrev;
        boolean bPressed = gamepad2.b && !bPrev;

        if (xPressed) {
            gate.setPosition(1.0);
            gateOpen = false;
        }

        if (bPressed) {
            servoSetter();
            gateOpen = true;
        }

        xPrev = gamepad2.x;
        bPrev = gamepad2.b;

        // ---------------- SHOOTING SEQUENCE ----------------
        if (gamepad2.dpad_up && !dpadUpPrevious) {
            shootAllBalls();
        }

        dpadUpPrevious = gamepad2.dpad_up;

        // ---------------- TELEMETRY ----------------
        telemetry.addData("Shooter Mode", shooterFarMode ? "FAR PID" : "CLOSE PID");
        telemetry.addData("Shooter Left Velocity", shooter.getLeftShooterVelocity());
        telemetry.addData("Shooter Right Velocity", shooter.getRightShooterVelocity());
        telemetry.addData("Intake Power", intakeTop.getPower());
        telemetry.addData("Intake Mode", intakeSlowMode ? "SLOW" : "FULL");
        telemetry.addData("Gate Position", gateOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Gate Raw Position", gate.getPosition());
        telemetry.addData("IMU Heading (Radians)", botHeading);
        telemetry.addData("Headless Mode", headlessEnabled ? "ON" : "OFF");
        telemetry.addData("Drive Speed Mode", speedDivisor == 1.0 ? "FULL" : "SLOW");
    }
}