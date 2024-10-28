package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

public class PIDController {

    public double kP, kI, kD, kF; // PID gains
    public double integral, previousError, derivative, error;
    private double targetAmount; // Desired value
    private double maxOutput = 0.5; // Maximum allowable motor power
    private double output;
    //private double rampRate = 0.1;// Rate at which the power ramps up
    public double tolerance = 50;

    public PIDController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.integral = 0;
        this.previousError = 0;
        this.targetAmount = 0;
    }

    public void setTargetAmount(double targetAmount) {
        this.targetAmount = targetAmount;
    }

    public double update(double currentValue, double deltaTime) {
        error = (targetAmount - currentValue);
        integral += error * deltaTime;
        //derivative = (error - previousError) / deltaTime;
        derivative = (error - previousError);
        double accelTicks = targetAmount/4;

        double baseOutput = (kP * error) + (kI * integral) + (kD * derivative);
        while (currentValue < accelTicks){
            output = kF + baseOutput * (currentValue/accelTicks);
            return clampOutput(output);
        }
        output = kF+ baseOutput;
        previousError = error;

        return clampOutput(output);
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