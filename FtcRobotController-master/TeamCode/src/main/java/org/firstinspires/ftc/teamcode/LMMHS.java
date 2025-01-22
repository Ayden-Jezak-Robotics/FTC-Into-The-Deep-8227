package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class LMMHS {

    private static final boolean inDegrees = true;

    public static double cos(double angle) {

        if (inDegrees) {
            return Math.cos(Math.toRadians(angle));
        } else {
            return Math.cos(angle);
        }

    }

    public static double sin(double angle) {

        if (inDegrees) {
            return Math.sin(Math.toRadians(angle));
        } else {
            return Math.sin(angle);
        }

    }

    public static AngleUnit getAngleUnit() {
        if (inDegrees) {
            return AngleUnit.DEGREES;
        } else {
            return AngleUnit.RADIANS;
        }

    }

    public static double setAngle(double initialAngle) {
        if (inDegrees) {
            return initialAngle;
        } else {
            return Math.toRadians(initialAngle);
        }
    }

    public static double normalizeAngle(double initialAngle) {
        if (inDegrees) {
            return ((initialAngle + 180) % 360 + 360) % 360 - 180;
        } else {
            return ((initialAngle + Math.PI) % (2 * Math.PI) + (2 * Math.PI)) % (2 * Math.PI) - Math.PI;
        }
    }

    public static double turnTolerance() {
        if (inDegrees) {
            return Constants.TURN_TOLERANCE_IN_DEGREES;
        } else {
            return Constants.TURN_TOLERANCE_IN_RADIANS;
        }

    }
}

