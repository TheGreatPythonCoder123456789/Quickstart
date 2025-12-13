package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem {
    private DcMotorEx shootLeft;
    private DcMotorEx shootRight;

    private static final int TICKS_PER_REV = 28;

    // Perfected baseline values
    private double baseP = 5.4;
    private double baseD = 0.7;
    private double baseFLeft = 12.95;
    private double baseFRight = 12.75;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shootLeft  = hardwareMap.get(DcMotorEx.class, "shootLeft");
        shootRight = hardwareMap.get(DcMotorEx.class, "shootRight");

        shootRight.setDirection(DcMotorEx.Direction.REVERSE);

        // Apply perfected PIDF
        PIDFCoefficients leftPIDF  = new PIDFCoefficients(baseP, 0.0, baseD, baseFLeft);
        PIDFCoefficients rightPIDF = new PIDFCoefficients(baseP, 0.0, baseD, baseFRight);

        shootLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, leftPIDF);
        shootRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, rightPIDF);
    }

    public void setShooterVelocity(double ticksPerSecond) {
        shootLeft.setVelocity(ticksPerSecond);
        shootRight.setVelocity(ticksPerSecond);
    }

    public void setTargetRPM(double rpm) {
        double ticksPerSecond = (rpm / 60.0) * TICKS_PER_REV;
        setShooterVelocity(ticksPerSecond);
    }

    public void stopShooter() {
        shootLeft.setVelocity(0);
        shootRight.setVelocity(0);
    }

    public double getLeftShooterVelocity() {
        return shootLeft.getVelocity();
    }

    public double getRightShooterVelocity() {
        return shootRight.getVelocity();
    }

    // Re-added: Update P/D live for TeleOp test
    public void updatePD(double pLeft, double dLeft, double pRight, double dRight) {
        PIDFCoefficients leftPIDF  = new PIDFCoefficients(pLeft, 0.0, dLeft, baseFLeft);
        PIDFCoefficients rightPIDF = new PIDFCoefficients(pRight, 0.0, dRight, baseFRight);

        shootLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, leftPIDF);
        shootRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, rightPIDF);
    }
}