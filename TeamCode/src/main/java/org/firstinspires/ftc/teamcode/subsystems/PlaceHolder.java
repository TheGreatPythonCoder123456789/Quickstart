/*
package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystemCloseShooting {

    private DcMotorEx shootLeft;
    private DcMotorEx shootRight;

    private static final int TICKS_PER_REV = 28;

    private double baseP = 14.2; //6.9
    private double baseD = 2.9; //1.1
    private double baseFLeft = 6.8; //12.3
    private double baseFRight = 7.2; //11.7

    public ShooterSubsystemCloseShooting(HardwareMap hardwareMap) {
        shootLeft  = hardwareMap.get(DcMotorEx.class, "shootLeft");
        shootRight = hardwareMap.get(DcMotorEx.class, "shootRight");

        shootRight.setDirection(DcMotorEx.Direction.REVERSE);

        applyPIDF();
    }

    private void applyPIDF() {
        PIDFCoefficients leftPIDF  = new PIDFCoefficients(baseP, 0.0, baseD, baseFLeft);
        PIDFCoefficients rightPIDF = new PIDFCoefficients(baseP, 0.0, baseD, baseFRight);

        shootLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, leftPIDF);
        shootRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, rightPIDF);
    }

    // ---------------- Shooter Control ----------------

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

    // ---------------- Live Tuning ----------------

    public void updatePD(double pLeft, double dLeft, double pRight, double dRight) {
        baseP = pLeft;
        baseD = dLeft;
        applyPIDF();
    }

    public void updatePD_official(double p, double d) {
        baseP = p;
        baseD = d;
        applyPIDF();
    }

    public void updateF(double fLeft, double fRight) {
        baseFLeft = fLeft;
        baseFRight = fRight;
        applyPIDF();
    }

    public double getFLeft() { return baseFLeft; }
    public double getFRight() { return baseFRight; }
    public double getP() { return baseP; }
    public double getD() { return baseD; }
}

 */