package org.firstinspires.ftc.teamcode;

public class Constants {

    public static final double DEAD_WHEEL_DIAMETER = 60;
    public static final double WHEEL_BASE_WIDTH = 130;

    private static final double DEAD_WHEEL_TICKS_PER_ROTATION = 8192;
    private static final double DEAD_WHEEL_CIRCUMFERENCE = (Math.PI * DEAD_WHEEL_DIAMETER);
    private static final double DEAD_WHEEL_CIRCUMFERENCE_INCH = (Math.PI * DEAD_WHEEL_DIAMETER) / 25.4;

    public static final double DEAD_WHEEL_TICKS_PER_INCH = DEAD_WHEEL_TICKS_PER_ROTATION / DEAD_WHEEL_CIRCUMFERENCE_INCH;
    public static final double DEAD_WHEEL_TICKS_PER_MM = DEAD_WHEEL_TICKS_PER_ROTATION / DEAD_WHEEL_CIRCUMFERENCE;
    public static final double DEAD_WHEEL_MM_PER_TICK = DEAD_WHEEL_CIRCUMFERENCE/DEAD_WHEEL_TICKS_PER_ROTATION;

    public static final double MINIMUM_TIME_IN_SECONDS = 1e-3;
    public static final double CONVERT_TIME_TO_SECONDS = 1000;
    public static final double MINIMUM_DISTANCE = 50;
    public static final double TURN_TOLERANCE = 1;

    public static final double APRIL_TAG_WEIGHT = 0.8;

//    public static final double TURN_SPEED = 0.2;

    public static final double MAX_INTEGRAL = 1000;

    public static final long MAX_AGE_NANOSECONDS = 250000000;

    private Constants() {
        throw new AssertionError("Cannot instantiate constants class");
    }
}

