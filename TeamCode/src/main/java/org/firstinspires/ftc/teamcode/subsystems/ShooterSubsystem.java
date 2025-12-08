package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem {
    private DcMotorEx shootLeft;
    private DcMotorEx shootRight;

    // Example PIDF coefficients (tune these!)
    private PIDFCoefficients pidf = new PIDFCoefficients(10.0, 3.0, 0.0, 12.0);

    // GoBilda 6000 RPM motor encoder ticks per revolution
    private static final int TICKS_PER_REV = 28;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shootLeft  = hardwareMap.get(DcMotorEx.class, "shootLeft");
        shootRight = hardwareMap.get(DcMotorEx.class, "shootRight");

        // Reverse one motor so both spin inward toward the ball
        shootRight.setDirection(DcMotorEx.Direction.REVERSE);

        // Apply PIDF coefficients
        shootLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        shootRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
    }

    /** Set shooter velocity in ticks per second */
    public void setShooterVelocity(double ticksPerSecond) {
        shootLeft.setVelocity(ticksPerSecond);
        shootRight.setVelocity(ticksPerSecond);
    }

    /** Set shooter velocity in RPM (converted to ticks/sec) */
    public void setTargetRPM(double rpm) {
        double ticksPerSecond = (rpm / 60.0) * TICKS_PER_REV;
        setShooterVelocity(ticksPerSecond);
    }

    /** Stop shooter motors */
    public void stopShooter() {
        shootLeft.setVelocity(0);
        shootRight.setVelocity(0);
    }

    /** Read current velocity (ticks/sec) from left motor */
    public double getLeftShooterVelocity() {
        return shootLeft.getVelocity();
    }

    /** Read current velocity (ticks/sec) from right motor */
    public double getRightShooterVelocity() {
        return shootRight.getVelocity();
    }
}