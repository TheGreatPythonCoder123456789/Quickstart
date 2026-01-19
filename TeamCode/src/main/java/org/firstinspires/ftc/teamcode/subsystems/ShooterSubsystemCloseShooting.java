package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShooterSubsystemCloseShooting {

    private DcMotorEx shootLeft;
    private DcMotorEx shootRight;

    private static final int TICKS_PER_REV = 28;

    // CLOSE PIDF (current)
    private double closeP = 14.2;
    private double closeD = 2.9;
    private double closeFLeft = 6.8;
    private double closeFRight = 7.2;

    // FAR PIDF (put your far-tuned values here)
    private double farP = 6.9;
    private double farD = 1.1;
    private double farFLeft = 12.3;
    private double farFRight = 11.7;

    // Active PIDF
    private double baseP = farP;
    private double baseD = farD;
    private double baseFLeft = farFLeft;
    private double baseFRight = farFRight;

    public ShooterSubsystemCloseShooting(HardwareMap hardwareMap) {
        shootLeft = hardwareMap.get(DcMotorEx.class, "shootLeft");
        shootRight = hardwareMap.get(DcMotorEx.class, "shootRight");

        shootRight.setDirection(DcMotorEx.Direction.REVERSE);

        applyPIDF();
    }

    private void applyPIDF() {
        PIDFCoefficients leftPIDF = new PIDFCoefficients(baseP, 0.0, baseD, baseFLeft);
        PIDFCoefficients rightPIDF = new PIDFCoefficients(baseP, 0.0, baseD, baseFRight);

        shootLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, leftPIDF);
        shootRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, rightPIDF);
    }

    // -------- Mode switching --------
    public void useClosePID() {
        baseP = closeP;
        baseD = closeD;
        baseFLeft = closeFLeft;
        baseFRight = closeFRight;
        applyPIDF();
    }

    public void useFarPID() {
        baseP = farP;
        baseD = farD;
        baseFLeft = farFLeft;
        baseFRight = farFRight;
        applyPIDF();
    }

    // -------- Shooter control --------
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

    // Optional: keep your live tuning methods if you want
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