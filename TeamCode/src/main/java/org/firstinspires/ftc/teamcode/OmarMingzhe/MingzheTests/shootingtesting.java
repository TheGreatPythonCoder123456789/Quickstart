package org.firstinspires.ftc.teamcode.OmarMingzhe.MingzheTests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemCloseShooting;

@TeleOp(name="<RPM> Shooting Test PIDF", group="TeleOp")
public class shootingtesting extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeTop;
    private IMU imu;

    private ShooterSubsystemCloseShooting shooter;

    // PIDF tuning variables
    private double currentP = 5;
    private double currentD = 0.7;

    // Independent F values
    private double currentFLeft = 12.5;
    private double currentFRight = 12.5;

    // Button edge tracking (PID)
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;
    private boolean dpadLeftPrev = false;
    private boolean dpadRightPrev = false;

    // Button edge tracking (F tuning)
    private boolean xPrev = false;
    private boolean bPrev = false;
    private boolean aPrev = false;
    private boolean yPrev = false;

    // Headless toggle
    private boolean headlessTogglePrev = false;
    private boolean headlessEnabled = true;

    // ---------------- Target RPM (Live Adjustable) ----------------
    private double targetRPM = 6000;
    private final double RPM_STEP = 20;

    // Gamepad1 RPM control edge detection
    private boolean x1Prev = false;
    private boolean b1Prev = false;

    @Override
    public void runOpMode() {
        initHardware();
        shooter = new ShooterSubsystemCloseShooting(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            runTeleop();
            telemetry.update();
        }
    }

    private void initHardware() {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        intakeTop  = hardwareMap.get(DcMotor.class, "intakeTop");
        imu        = hardwareMap.get(IMU.class, "imu");

        intakeTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    private void runTeleop() {

        // ---------------- Headless Drive ----------------
        double drive = -gamepad1.left_stick_y / 1.5;
        double strafe = gamepad1.left_stick_x / 1.5;
        double turn = gamepad1.right_stick_x;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (gamepad1.a) imu.resetYaw();
        if (gamepad1.y && !headlessTogglePrev) headlessEnabled = !headlessEnabled;
        headlessTogglePrev = gamepad1.y;

        if (headlessEnabled) {
            double rotX = strafe * Math.cos(-heading) - drive * Math.sin(-heading);
            double rotY = strafe * Math.sin(-heading) + drive * Math.cos(-heading);
            strafe = rotX;
            drive = rotY;
        }

        double fl = drive + strafe + turn / 2;
        double fr = drive - strafe - turn / 2;
        double bl = drive - strafe + turn / 2;
        double br = drive + strafe - turn / 2;

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);

        // ---------------- Gamepad1 RPM Control ----------------
        if (gamepad1.x && !x1Prev) {
            targetRPM = Math.max(0, targetRPM - RPM_STEP);
        }

        if (gamepad1.b && !b1Prev) {
            targetRPM += RPM_STEP;
        }

        x1Prev = gamepad1.x;
        b1Prev = gamepad1.b;

        // ---------------- Intake ----------------
        if (gamepad2.left_bumper) intakeTop.setPower(-1);
        else if (gamepad2.right_bumper) intakeTop.setPower(1);
        else intakeTop.setPower(0);

        // ---------------- Shooter ----------------
        if (gamepad2.right_trigger > 0) {
            shooter.setTargetRPM(targetRPM);
        } else {
            shooter.stopShooter();
        }

        // ---------------- Live P & D Tuning ----------------
        if (gamepad2.dpad_up && !dpadUpPrev) currentP += 0.1;
        if (gamepad2.dpad_down && !dpadDownPrev) currentP = Math.max(0, currentP - 0.1);

        if (gamepad2.dpad_right && !dpadRightPrev) currentD += 0.1;
        if (gamepad2.dpad_left && !dpadLeftPrev) currentD = Math.max(0, currentD - 0.1);

        shooter.updatePD_official(currentP, currentD);

        dpadUpPrev = gamepad2.dpad_up;
        dpadDownPrev = gamepad2.dpad_down;
        dpadLeftPrev = gamepad2.dpad_left;
        dpadRightPrev = gamepad2.dpad_right;

        // ---------------- Live F Tuning ----------------
        if (gamepad2.b && !bPrev) currentFLeft += 0.1;
        if (gamepad2.x && !xPrev) currentFLeft = Math.max(0, currentFLeft - 0.1);

        if (gamepad2.y && !yPrev) currentFRight += 0.1;
        if (gamepad2.a && !aPrev) currentFRight = Math.max(0, currentFRight - 0.1);

        shooter.updateF(currentFLeft, currentFRight);

        bPrev = gamepad2.b;
        xPrev = gamepad2.x;
        yPrev = gamepad2.y;
        aPrev = gamepad2.a;

        // ---------------- Telemetry ----------------
        double leftVel = shooter.getLeftShooterVelocity();
        double rightVel = shooter.getRightShooterVelocity();
        double avgVel = (leftVel + rightVel) / 2.0;

        double targetTicksPerSec = (targetRPM / 60.0) * 28;

        telemetry.addData("Shooter Left Vel", leftVel);
        telemetry.addData("Shooter Right Vel", rightVel);
        telemetry.addData("Avg Vel", avgVel);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Target Ticks/sec", targetTicksPerSec);
        telemetry.addData("Error", targetTicksPerSec - avgVel);

        telemetry.addData("P", currentP);
        telemetry.addData("D", currentD);
        telemetry.addData("F Left", currentFLeft);
        telemetry.addData("F Right", currentFRight);

        telemetry.addLine("Shooter PIDF Tuning Guide:");
        telemetry.addLine("- Set P and D to zero temporarily");
        telemetry.addLine("- Increase F until the shooter reaches target RPM");
        telemetry.addLine("- Add P until it stabilizes quickly");
        telemetry.addLine("- Add D only if oscillation appears");
    }
}