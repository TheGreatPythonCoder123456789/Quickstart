package org.firstinspires.ftc.teamcode;
//for positioning robot make it on red tape by alligning it with
// the right and left ends of the C channels (end of the C channels)

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name="TeleopHeadlessAndroidStudio", group="TeleOp")
public class teleopHeadlessAndroidStudio extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intakeTop;
    private IMU imu;

    // Shooter subsystem
    private ShooterSubsystem shooter;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        // Initialize shooter subsystem
        shooter = new ShooterSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            runTeleop();
            telemetry.update();
        }
    }

    boolean dpadUpPrevious = false;
    boolean yButtonPrevious = false;
    boolean dpadDownPrevious = false;
    boolean headlessEnabled = true;
    double botHeading = 0.0;

    private void initializeHardware() {
        // Map drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Map mechanism motors
        intakeTop = hardwareMap.get(DcMotor.class, "intakeTop");

        // Map IMU
        imu = hardwareMap.get(IMU.class, "imu");

        intakeTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }

    private void shootMechLast() { }
    private void shootfrst2Balls() { }

    private void runTeleop() {
        // Get joystick input
        double drive = -gamepad1.left_stick_y / 1.5;
        double strafe = gamepad1.left_stick_x / 1.5;
        double turn  = gamepad1.right_stick_x;

        // Headless mode
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

        double frontLeftPower  = drive + strafe + turn / 2;
        double frontRightPower = drive - strafe - turn / 2;
        double backLeftPower   = drive - strafe + turn / 2;
        double backRightPower  = drive + strafe - turn / 2;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // Intake control
        if (gamepad2.left_bumper) {
            intakeTop.setPower(-1.0);
        } else if (gamepad2.right_bumper) {
            intakeTop.setPower(1.0);
        } else {
            intakeTop.setPower(0);
        }

        // Shooter control using subsystem
        if (gamepad2.right_trigger > 0) {
            shooter.setTargetRPM(2100); // example target RPM
        } else {
            shooter.stopShooter();
        }

        if (gamepad2.dpad_up && !dpadUpPrevious) {
            shootfrst2Balls();
            shootMechLast();
        }
        dpadUpPrevious = gamepad2.dpad_up;

        // Telemetry
        telemetry.addData("Shooter Left Velocity", shooter.getLeftShooterVelocity());
        telemetry.addData("Shooter Right Velocity", shooter.getRightShooterVelocity());
        telemetry.addData("intakeTop Power", intakeTop.getPower());
        telemetry.addData("Front Left", frontLeft.getPower());
        telemetry.addData("Front Right", frontRight.getPower());
        telemetry.addData("Back Left", backLeft.getPower());
        telemetry.addData("Back Right", backRight.getPower());
        telemetry.addData("IMU Heading (Radians)", botHeading);
        telemetry.addData("Headless Mode", headlessEnabled ? "ON" : "OFF");
        telemetry.addData("Y Button State", gamepad1.y);
    }
}