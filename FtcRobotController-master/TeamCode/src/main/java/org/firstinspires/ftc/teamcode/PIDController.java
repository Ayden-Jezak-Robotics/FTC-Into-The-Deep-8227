package org.firstinspires.ftc.teamcode;

public class PIDController {

    private double kP, kI, kD; // PID gains
    private double integral, previousError;
    private double targetAmount; // Desired value
    private double maxOutput = 0.5; // Maximum allowable motor power
    private double rampRate = 0.25; // Rate at which the power ramps up

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integral = 0;
        this.previousError = 0;
        this.targetAmount = 0;
    }

    public void setTargetAmount(double targetAmount) {
        this.targetAmount = targetAmount;
    }

    public double update(double currentValue, double deltaTime) {
        double error = (targetAmount - currentValue);
        integral += error * deltaTime;
        double derivative = (error - previousError) / deltaTime;

        double baseOutput = (kP * error) + (kI * integral) + (kD * derivative);

        double output = rampUpOutput(baseOutput, deltaTime);

        previousError = error;
        return clampOutput(output);
    }

   private double rampUpOutput(double baseOutput, double deltaTime) {
        double targetOutput = baseOutput;
        double currentOutput = previousOutput;

        // Calculate the maximum change in output based on the ramp rate
        rampRate = .2;
        double maxChange = rampRate * deltaTime;

        // Calculate the new output by ramping up
        if (targetOutput > currentOutput) {
            currentOutput += Math.min(maxChange, targetOutput - currentOutput);
        } else {
            currentOutput = targetOutput; // Directly set to target if ramping down
        }

        previousOutput = currentOutput; // Update the previous output
        return currentOutput;
    }


    public double clampOutput(double output) {
        return Math.max(-maxOutput, Math.min(maxOutput, output));
    }

    public void reset() {
        integral = 0;
        previousError = 0;
        previousOutput = 0; // Reset the previous output as well
    }

    private double previousOutput = 0; // Field to store the previous output
}