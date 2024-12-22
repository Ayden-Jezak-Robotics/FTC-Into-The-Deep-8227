package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class SanjuPIDController {
    private double kP, kI, kD, kF;
    private double derivative;
    private double originalError; //in ticks
    private double integralSum;
    private double error; // in ticks
    private double lastError;
    // Adjust based on your system's needs

    public SanjuPIDController(double kP, double kI, double kD, double kF)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.integralSum = 0;
        this.lastError = 0;
    }

    public SanjuPIDController(PIDType type)
    {
        switch (type) {
            case STRAIGHT:
                this.kP = 0.00003;
                this.kI = 0;
                this.kD = 0.004;
                this.kF = 0.2;
                break;
            case STRAFE:
                this.kP = 0.00003;
                this.kI = 0;
                this.kD = 0.004;
                this.kF = 0.22;
                break;
            case TURN:
                this.kP = 0.003;
                this.kI = 0;
                this.kD = 0;
                this.kF = 0.15;
                break;
            default:
        }

        this.integralSum = 0;
        this.lastError = 0;
    }

    public void setTarget(double target)
    {
        this.originalError = target;
    }

    public void setError(double newError)
    {
        error = newError;
    }

    public double calculateOutput(double currentPosition, double deltaTime)
    {
        error = originalError - currentPosition;

        double aMaxPoint = originalError /3;

        double proportional = kP * error;

        integralSum += (error * deltaTime);
        // Anti-windup especially when distance is large
        integralSum = Range.clip(integralSum, - Constants.MAX_INTEGRAL, Constants.MAX_INTEGRAL);
        double integral = kI * integralSum;

        double feedforward = kF * Math.signum(error);

        derivative = kD * (error - lastError)/deltaTime;

        lastError = error;

        double baseOutput = proportional + integral + derivative;
        double output;

        if (Math.abs(originalError) < 6600 && originalError != 0)
        {
            output = 0.30 * Math.signum(originalError);
        }
        else if (originalError <0)
        {
            if (currentPosition > aMaxPoint)
            {
                output = feedforward + (currentPosition/aMaxPoint) * baseOutput;
            }
            else
            {
                output = feedforward + baseOutput;
            }
        }
        else if (originalError > 0)
        {
            if (currentPosition < aMaxPoint)
            {
                output = feedforward + (currentPosition/aMaxPoint) * baseOutput;
            }
            else
            {
                output = feedforward + baseOutput;
            }
        }
        else
        {
            output = feedforward + baseOutput;
        }

//       output = Range.clip(output, -1, 1);

        return output;
    }

    public double getDerivative() {
        return derivative;
    }

    public double updateError(double target, double newLoc)
    {
        error = target - newLoc;
        return error;
    }

    public double getError()
    {
        return error;
    }
    public double getTarget()
    {
        return originalError;
    }
    public double getkF()
    {
        return kF;
    }
    public void reset()
    {
        integralSum = 0;
        lastError = 0;
    }

}