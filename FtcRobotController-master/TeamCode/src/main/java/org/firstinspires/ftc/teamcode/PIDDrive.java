package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class PIDDrive {
    private static final double STRAIGHT_KP = 0.0001;
    private static final double STRAIGHT_KI = 0.0000568;
    private static final double STRAIGHT_KD = 0.00003;
    private static final double STRAIGHT_KF = 0;

    private static final double STRAFE_KP = 0.0001;
    private static final double STRAFE_KI = 0.0000568;
    private static final double STRAFE_KD = 0.00003;
    private static final double STRAFE_KF = 0;

    private static final kValues straight = new kValues(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD, STRAIGHT_KF);
    private static final kValues strafe = new kValues(STRAFE_KP, STRAFE_KI, STRAFE_KD, STRAFE_KF);

    private Position targetPosition;

    private XYValue priorError;
    private XYValue integralSum;

    private Telemetry telemetry;

    public PIDDrive(Telemetry telemetry) {

        this.priorError = new XYValue(0, 0);
        this.integralSum = new XYValue(0, 0);

        this.telemetry = telemetry;

    }

    public void setTargetPosition(Position newTarget) {

        this.targetPosition = newTarget;

    }

    /// currentPosition in Inches; currentHeading in Radians
    public XYValue calculateLocalError(Position currentPosition, double currentHeading) {
        XYValue localError = new XYValue(0, 0);

        double tempErrorX = (targetPosition.x - currentPosition.x) * LMMHS.cos(currentHeading) + (targetPosition.y - currentPosition.y) * LMMHS.sin(currentHeading);
        double tempErrorY = (targetPosition.y - currentPosition.y) * LMMHS.cos(currentHeading) - (targetPosition.x - currentPosition.x) * LMMHS.sin(currentHeading);

        localError.x = tempErrorX * Constants.DEAD_WHEEL_TICKS_PER_INCH;
        localError.y = tempErrorY * Constants.DEAD_WHEEL_TICKS_PER_INCH;

        return localError;
    }

    /// currentPosition in Inches; currentHeading in Radians; time in Seconds
    public XYValue calculatePower(Position currentPosition, double currentHeading, double time) {
        XYValue error = calculateLocalError(currentPosition, currentHeading);

        /// Prevent zero or very small time steps
        double deltaTime = Math.max(time, Constants.MINIMUM_TIME_IN_SECONDS);

        double kProportionalValueX = Range.clip(strafe.kP * error.x, -Constants.MAX_KP, Constants.MAX_KP);
        double kProportionalValueY = Range.clip(straight.kP * error.y, -Constants.MAX_KP, Constants.MAX_KP);

        // Decay the Integral Sum over time
        integralSum.x = (integralSum.x * 0.98) + (error.x * deltaTime);
        integralSum.y = (integralSum.y * 0.98) + (error.y * deltaTime);

        double kIntegralValueX = strafe.kI * Range.clip(integralSum.x, -Constants.MAX_INTEGRAL_XY, Constants.MAX_INTEGRAL_XY);
        double kIntegralValueY = straight.kI * Range.clip(integralSum.y, -Constants.MAX_INTEGRAL_XY, Constants.MAX_INTEGRAL_XY);

        double kDerivativeValueX = strafe.kD * ((error.x - priorError.x) / deltaTime);
        double kDerivativeValueY = straight.kD * ((error.y - priorError.y) / deltaTime);

        double kFeedForwardValueX = strafe.kF * Math.signum(error.x);
        double kFeedForwardValueY = straight.kF * Math.signum(error.y);

        priorError = error;

        XYValue baseOutputXY = new XYValue(0, 0);

        baseOutputXY.x = kProportionalValueX + kIntegralValueX + kDerivativeValueX + kFeedForwardValueX;
        baseOutputXY.y = kProportionalValueY + kIntegralValueY + kDerivativeValueY + kFeedForwardValueY;

/*

        if (baseOutput.x < Constants.MINIMUM_POWER_OUTPUT_DRIVE) {
            baseOutput.x = Math.signum(baseOutput.x) * Constants.MINIMUM_POWER_OUTPUT_DRIVE;
        }

        if (baseOutput.y < Constants.MINIMUM_POWER_OUTPUT_DRIVE) {
            baseOutput.y = Math.signum(baseOutput.y) * Constants.MINIMUM_POWER_OUTPUT_DRIVE;
        }

*/

        return baseOutputXY;
    }
}
