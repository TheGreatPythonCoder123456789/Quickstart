package org.firstinspires.ftc.teamcode.OmarMingzhe.MingzheTests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="GG1")
public class GG1 extends LinearOpMode {

    // Motors
    private DcMotor LF, RF, LB, RB, Servo0;
    private DcMotor ShooterL, ShooterR;
    private IMU imu;

    public static final double Pticks = 5, Pyaw = 2;
    public static final double TICKS_PER_REV = 384.0;
    public static final double WHEEL_DIAMETER_CM = 9.6;
    public static final double WHEEL_CIRCUMFERENCE_CM = Math.PI * WHEEL_DIAMETER_CM; // ≈ 30.15928947
    public static final double TICKS_PER_CM = TICKS_PER_REV / WHEEL_CIRCUMFERENCE_CM; // ≈ 12.73239545

    // PID gains (starting values — already reasonable for 312RPM + 96mm)

    private double kP_xy = 0.015;
    private double kI_xy = 0.0;
    private double kD_xy = 0.0006;

    private double kP_yaw = 0.02;
    private double kI_yaw = 0.0;
    private double kD_yaw = 0.001;

    // PID state
    private double integralX = 0, lastErrorX = 0;
    private double integralY = 0, lastErrorY = 0;
    private double integralYaw = 0, lastErrorYaw = 0;
    private double imuu = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        // hardware map
        Init();
        waitForStart();
        int flag = 1;
        boolean last = false;
        while (opModeIsActive()) {
            if (gamepad1.a) {
                // ShooterR.setPower(0.1);

                ShooterR.setPower(0.2 * flag);
            } else {
                ShooterR.setPower(0);
                // ShooterR.setVelocity(0);
            }
            if (gamepad1.x) {
                flag = -flag;
            }
            if (gamepad1.b) {
                if (!last) {
                    // Ready();
                    // ShooterL.setVelocity(3000);
                    ShooterL.setPower(1.0);
                    // ShooterL.setPower(0.5);
                    //                ShooterR.setVelocity(1400);
                } else {
                    ShooterL.setPower(0);
                    // ShooterL.setVelocity(0);
                    //                ShooterR.setVelocity(0);
                }
                last = !last;
                //            setMotorPower(speed,-gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x);
            }

            telemetry.addData("StatusL", last);
//            telemetry.addData("StatusR", ShooterR.getVelocity());
            telemetry.update();
            sleep(200);
        }
//        driveToPositionPID(0,-120,mp(imuu-15),6,0.5);
//        sleep(200);
//        Ready();
//        Work2(1,0.3);
//        sleep(3000);
//        Work2(-0.4,0.4);
        // driveToPositionPID(0, -40, 10, 100,0.4);

//        stopAll();
    }

    // Main PID drive method
    // targetY: forward/back ticks (positive = forward)
    // targetX: right/left ticks (positive = right strafing)
    // targetYaw: desired absolute yaw in degrees (IMU yaw)
    // timeoutMs: safety timeout
    private void Init() {

        ShooterL = hardwareMap.get(DcMotor.class, "shooter");
        ShooterR = hardwareMap.get(DcMotor.class, "rotation");
//        Servo0=hardwareMap.get(DcMotor.class, "getba0");
//
//        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ShooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Servo1=hardwareMap.get(Servo.class, "getba1");
//        Servo1.setDirection(Servo.Direction.FORWARD);
//        Servo1.setPosition(0.4);
        // Work3(0.5);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(imuParams);
        imuu = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


        // LB.setDirection(DcMotor.Direction.REVERSE);
        // LF.setDirection(DcMotor.Direction.REVERSE);
        ShooterL.setDirection(DcMotor.Direction.FORWARD);
        ShooterR.setDirection(DcMotor.Direction.REVERSE);
        // ShooterL.setVelocityPIDFCoefficients(0.17,0.0,0.0,12.2);
        // ShooterR.setVelocityPIDFCoefficients(0.17,0.0,0.0,12.2);

        telemetry.addData("Status", imuu);
        telemetry.update();

    }

    public void driveToPositionPID(double X, double Y, double targetYaw, double time, double maxSpeed) {
        int targetX = (int) Math.round(X * TICKS_PER_CM);
        int targetY = (int) Math.round(Y * TICKS_PER_CM);
        // targetYaw=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+targetYaw;
        // targetYaw=mp(targetYaw);
        double lastTime = 0;
        double integralX, integralY;
        double lastErrorX, lastErrorY;
        integralX = integralY = integralYaw = 0;
        lastErrorX = lastErrorY = lastErrorYaw = 0;

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (opModeIsActive()) {
            double now = runtime.seconds();
            double dt = now - lastTime;
            lastTime = now;

            int lf = LF.getCurrentPosition();
            int rf = RF.getCurrentPosition();
            int lb = LB.getCurrentPosition();
            int rb = RB.getCurrentPosition();

            double nowY = 1.0 * (lf + rf + lb + rb) / 4;
            double nowX = 1.0 * (lf - rf - lb + rb) / 4;

            double erY = targetY - nowY;
            double erX = targetX - nowX;

            // Y PID
            integralY += erY * dt;
            double derivativeY = (erY - lastErrorY) / dt;
            double outputY = kP_xy * erY + kI_xy * integralY + kD_xy * derivativeY;
            lastErrorY = erY;

            // X PID
            integralX += erX * dt;
            double derivativeX = (erX - lastErrorX) / dt;
            double outputX = kP_xy * erX + kI_xy * integralX + kD_xy * derivativeX;
            lastErrorX = erX;

            // Yaw PID (IMU)
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double erYaw = mp(yaw - targetYaw); // shortest path
            integralYaw += erYaw * dt;
            double derivativeYaw = (erYaw - lastErrorYaw) / dt;
            double outputYaw = kP_yaw * erYaw + kI_yaw * integralYaw + kD_yaw * derivativeYaw;
            lastErrorYaw = erYaw;

            double lfPower = outputY + outputX + outputYaw;
            double rfPower = outputY - outputX - outputYaw;
            double lbPower = outputY - outputX + outputYaw;
            double rbPower = outputY + outputX - outputYaw;

            // Normalize to [-1,1]
            double max = Math.max(1.0, Math.max(
                    Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                    Math.max(Math.abs(lbPower), Math.abs(rbPower))));
            lfPower /= max;
            rfPower /= max;
            lbPower /= max;
            rbPower /= max;

            // Apply small cap for safety / smoother motion
//            double maxSpeed = 0.4; // tune if needed
            LF.setPower(lfPower * maxSpeed);
            RF.setPower(rfPower * maxSpeed);
            LB.setPower(lbPower * maxSpeed);
            RB.setPower(rbPower * maxSpeed);

//            double Pticks = 8.0;
//            double Pyaw = 1.5;     // degrees tolerance

//            telemetry.addData("er", erYaw);
            telemetry.addData("yaw", yaw);
//            telemetry.addData("target", targetYaw);
            // sleep(3000);
            telemetry.update();

            if (Math.max(Math.abs(erX), Math.abs(erY)) < Pticks && Math.abs(erYaw) < Pyaw) break;
            if (runtime.seconds() > time) break;
        }
        stopAll();
    }

    private void stopAll() {
        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
    }

    // normalize angle to [-180, 180)
    private double mp(double angle) {
        return (angle + 540) % 360 - 180;
    }

    public void Work2(double speed, double time) {
        ElapsedTime runtime1 = new ElapsedTime();
        runtime1.reset();
        stopAll();
        while (opModeIsActive()) {
//            Servo2.setPower(speed);
            sleep(200);
            Servo0.setPower(speed);
            if (runtime1.seconds() >= time) {
                break;
            }
            sleep(30);
        }
//        Servo2.setPower(0);
        Servo0.setPower(0);
    }
    /*
    private void Ready(){
        while(!Readytoshoot()){
            ShooterL.setVelocity(1400);
            ShooterR.setVelocity(1400);
            telemetry.addData("Status", 0.5*(ShooterL.getVelocity()+ShooterR.getVelocity()));
            telemetry.addData("StatusL", ShooterL.getVelocity());
            telemetry.addData("StatusR", ShooterR.getVelocity());
            telemetry.update();
        }
    }
    private boolean Readytoshoot(){
        double gg=0.5*(ShooterL.getVelocity()+ShooterR.getVelocity());
        if(1350<=gg&&gg<=1450){
            return true;
        }
        return false;
    }
}

*/
}
