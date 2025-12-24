package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name="TEST_headlessAndroid_PID_SHOOTER", group="TeleOp")
public class teleopHeadlessAndroidStudioTest extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intakeTop;
    private IMU imu;

    private ShooterSubsystem shooter;

    // PID tuning variables (per motor)
    private double currentPLeft = 5.4;
    private double currentDLeft = 0.7;
    private double currentPRight = 5.4;
    private double currentDRight = 0.7;

    private final double targetRPM = 2100;
    private final double targetTicksPerSec = (targetRPM / 60.0) * 28; // ≈980

    private boolean dpadUpPrevious = false;
    private boolean dpadDownPrevious = false;
    private boolean yButtonPrevious = false;
    private boolean headlessEnabled = true;
    private double botHeading = 0.0;

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
        imu = hardwareMap.get(IMU.class, "imu");

        intakeTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }

    private void runTeleop() {
        double drive = -gamepad1.left_stick_y / 1.5;
        double strafe = gamepad1.left_stick_x / 1.5;
        double turn  = gamepad1.right_stick_x;

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

        // Shooter control
        if (gamepad2.right_trigger > 0) {
            shooter.setTargetRPM(targetRPM);
        } else {
            shooter.stopShooter();
        }

        // Live PID tuning with D-Pad (adjust both motors together)
        if (gamepad2.dpad_up && !dpadUpPrevious) {
            currentPLeft += 0.1;
            currentPRight += 0.1;
        }
        if (gamepad2.dpad_down && !dpadDownPrevious) {
            currentPLeft = Math.max(0.0, currentPLeft - 0.1);
            currentPRight = Math.max(0.0, currentPRight - 0.1);
        }
        if (gamepad2.dpad_right) {
            currentDLeft += 0.1;
            currentDRight += 0.1;
        }
        if (gamepad2.dpad_left) {
            currentDLeft = Math.max(0.0, currentDLeft - 0.1);
            currentDRight = Math.max(0.0, currentDRight - 0.1);
        }

        shooter.updatePD(currentPLeft, currentDLeft, currentPRight, currentDRight);

        dpadUpPrevious = gamepad2.dpad_up;
        dpadDownPrevious = gamepad2.dpad_down;

        // Telemetry
        double leftVel = shooter.getLeftShooterVelocity();
        double rightVel = shooter.getRightShooterVelocity();
        double avgVel = (leftVel + rightVel) / 2.0;
        double error = targetTicksPerSec - avgVel;

        telemetry.addData("Shooter Left Velocity", leftVel);
        telemetry.addData("Shooter Right Velocity", rightVel);
        telemetry.addData("Shooter Avg Velocity", avgVel);
        telemetry.addData("Target Ticks/sec", targetTicksPerSec);
        telemetry.addData("Velocity Error", error);
        telemetry.addData("Shooter Left P", currentPLeft);
        telemetry.addData("Shooter Left D", currentDLeft);
        telemetry.addData("Shooter Right P", currentPRight);
        telemetry.addData("Shooter Right D", currentDRight);
    }
}