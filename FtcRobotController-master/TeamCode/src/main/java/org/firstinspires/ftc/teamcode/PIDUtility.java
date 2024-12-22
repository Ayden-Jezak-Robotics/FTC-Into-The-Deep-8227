package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class PIDUtility {
    private PIDType type;

    private double kP, kI, kD, kF;

    private double initialPosition; // in inches
    private double targetPosition; // in inches
    private double originalError; //in ticks

    private double aMaxPoint; // in ticks
    private double integralSum;
    private double priorError;
    // Adjust based on your system's needs

    public PIDUtility(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.integralSum = 0;
        this.priorError = 0;
    }

    public PIDUtility(PIDType type) {
        this.type = type;

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
        this.priorError = 0;
    }

    public void setOriginalError(double initialPosition, double targetPosition) {
        this.initialPosition = initialPosition;
        this.targetPosition = targetPosition;

        this.originalError = (targetPosition - initialPosition);

        if (originalError == 0) {
            originalError = .006;
        }

        if (type == PIDType.STRAIGHT || type == PIDType.STRAFE) {
            this.originalError = this.originalError * Constants.DEAD_WHEEL_TICKS_PER_INCH;
        }

        this.aMaxPoint = originalError / 3;
    }

    public double calculatePower(double currentPosition, double time) // time is in Milliseconds
    {
        double error;

        if (type == PIDType.STRAIGHT || type == PIDType.STRAFE) {
            error = (targetPosition - currentPosition) * Constants.DEAD_WHEEL_TICKS_PER_INCH;

            if (Math.abs(error) < Constants.MINIMUM_DISTANCE) {
                return 0;
            }
        } else { // For Turn based calculations
            double rawError = targetPosition - currentPosition;

            // Normalize error to the range [-180, 180]

            if (rawError > 180) {
                error = rawError - 360;
            } else if (rawError < -180) {
                error = rawError + 360;
            } else {
                error = rawError;
            }

            if (Math.abs(error) < Constants.TURN_TOLERANCE) {
                return 0;
            }
        }

        double deltaTime;

        if (time < Constants.MINIMUM_TIME_IN_SECONDS) {
            deltaTime = Constants.MINIMUM_TIME_IN_SECONDS; // Prevent zero or very small time steps
        } else {
            deltaTime = time / Constants.CONVERT_TIME_TO_SECONDS;
        }

        integralSum += (error * deltaTime);

        double kProportionalValue = kP * error;

        double kIntegralValue = kI * Range.clip(integralSum, -Constants.MAX_INTEGRAL, Constants.MAX_INTEGRAL);

        double kDerivativeValue = kD * ((error - priorError) / deltaTime);

        double kFeedForwardValue = kF * Math.signum(error);

        priorError = error;

        double baseOutput = kProportionalValue + kIntegralValue + kDerivativeValue;
        double errorCompleted;

        if (type == PIDType.STRAIGHT || type == PIDType.STRAFE) {
            if (Math.abs(originalError) < 6600 ) {
                return 0.30 * Math.signum(originalError);
            }

            errorCompleted = (currentPosition - initialPosition) * Constants.DEAD_WHEEL_TICKS_PER_INCH;


        } else {
            if (Math.abs(originalError) < 20 && originalError != 0) {
                return 0.30 * Math.signum(originalError);
            }

            errorCompleted = (currentPosition - initialPosition);

        }

        if (Math.abs(errorCompleted) < Math.abs(aMaxPoint)) {
            return kFeedForwardValue + (currentPosition / aMaxPoint) * baseOutput;
        } else {
            return kFeedForwardValue + baseOutput;
        }
    }
}
