package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem {
    private DcMotorEx shootLeft;
    private DcMotorEx shootRight;

    private static final int TICKS_PER_REV = 28;

    // Start with P=0, I=0, D=0 while tuning F
    private PIDFCoefficients leftPIDF  = new PIDFCoefficients(5.0, 0.0, 0.7, 12.95); // tuned from telemetry
    private PIDFCoefficients rightPIDF = new PIDFCoefficients(5.0, 0.0, 0.7, 12.75); // tuned from telemetry
                                                                                                //was 12.5 make 6 if bad low
    public ShooterSubsystem(HardwareMap hardwareMap) {
        shootLeft  = hardwareMap.get(DcMotorEx.class, "shootLeft");
        shootRight = hardwareMap.get(DcMotorEx.class, "shootRight");

        shootRight.setDirection(DcMotorEx.Direction.REVERSE);

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

    // Helper to adjust only F terms during tuning
    public void setFeedforward(double fLeft, double fRight) {
        leftPIDF  = new PIDFCoefficients(leftPIDF.p, leftPIDF.i, leftPIDF.d, fLeft);
        rightPIDF = new PIDFCoefficients(rightPIDF.p, rightPIDF.i, rightPIDF.d, fRight);
        shootLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, leftPIDF);
        shootRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, rightPIDF);
    }

    // Optional: set full PIDF if needed later
    public void setPIDF(double p, double i, double d, double fLeft, double fRight) {
        leftPIDF  = new PIDFCoefficients(p, i, d, fLeft);
        rightPIDF = new PIDFCoefficients(p, i, d, fRight);
        shootLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, leftPIDF);
        shootRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, rightPIDF);
    }
}