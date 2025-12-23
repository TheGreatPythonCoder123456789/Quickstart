package org.firstinspires.ftc.teamcode;

//for positioning robot make it on red tape by alligning it with
// the right and left ends of the C channels (end of the C channels)

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name="OFFICIAL_headlessAndroid", group="TeleOp")
public class teleopHeadlessAndroidStudio extends LinearOpMode {

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
    boolean yButtonPrevious = false;      // for gamepad1.y (headless)
    boolean gateYPrevious = false;        // for gamepad2.y (gate toggle)

    boolean headlessEnabled = true;
    boolean gateOpen = false;

    double botHeading = 0.0;

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
    }

    private void shootMechLast() { }
    private void shootfrst2Balls() { }

    private void runTeleop() {

        // ------------------ DRIVETRAIN ------------------
        double drive = -gamepad1.left_stick_y / 1.5;
        double strafe = gamepad1.left_stick_x / 1.5;
        double turn  = gamepad1.right_stick_x;

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (gamepad1.a) imu.resetYaw();

        // Toggle headless mode on gamepad1.y
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

        // ------------------ INTAKE ------------------
        if (gamepad2.left_bumper) {
            intakeTop.setPower(-1.0);
        } else if (gamepad2.right_bumper) {
            intakeTop.setPower(1.0);
        } else {
            intakeTop.setPower(0);
        }

        // ------------------ SHOOTER ------------------
        if (gamepad2.right_trigger > 0) {
            shooter.setTargetRPM(2100);
        } else {
            shooter.stopShooter();
        }

        // ------------------ GATE TOGGLE (GAMEPAD 2 Y) ------------------
        if (gamepad2.y && !gateYPrevious) {
            gateOpen = !gateOpen;

            if (gateOpen) {
                gate.setPosition(1.0);
            } else {
                gate.setPosition(0);
            }
        }
        gateYPrevious = gamepad2.y;

        // ------------------ SHOOTING SEQUENCE ------------------
        if (gamepad2.dpad_up && !dpadUpPrevious) {
            shootfrst2Balls();
            shootMechLast();
        }
        dpadUpPrevious = gamepad2.dpad_up;

        // ------------------ TELEMETRY ------------------
        telemetry.addData("Shooter Left Velocity", shooter.getLeftShooterVelocity());
        telemetry.addData("Shooter Right Velocity", shooter.getRightShooterVelocity());
        telemetry.addData("Intake Power", intakeTop.getPower());
        telemetry.addData("Gate Position", gateOpen ? "OPEN (1.0)" : "CLOSED (0.0)");
        telemetry.addData("IMU Heading (Radians)", botHeading);
        telemetry.addData("Headless Mode", headlessEnabled ? "ON" : "OFF");
    }
}