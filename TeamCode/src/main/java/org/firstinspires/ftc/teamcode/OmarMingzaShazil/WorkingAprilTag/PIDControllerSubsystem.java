
package org.firstinspires.ftc.teamcode.OmarMingzaShazil.WorkingAprilTag;

public class PIDControllerSubsystem {
    private double kP, kI, kD;
    private double integral = 0;
    private double lastError = 0;

    public PIDControllerSubsystem(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double error, double dt) {
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;
        return kP * error + kI * integral + kD * derivative;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
    }
}