package org.firstinspires.ftc.teamcode;

public class Constants {

    public static final double DEAD_WHEEL_DIAMETER = 32;
    public static final double WHEEL_BASE_WIDTH_MM = 90;
    public static final double WHEEL_BASE_LENGTH_MM = 180; //Double check the radii


    private static final double DEAD_WHEEL_TICKS_PER_ROTATION = 2000;
    private static final double DEAD_WHEEL_CIRCUMFERENCE = (Math.PI * DEAD_WHEEL_DIAMETER);
    private static final double DEAD_WHEEL_CIRCUMFERENCE_INCH = (Math.PI * DEAD_WHEEL_DIAMETER) / 25.4;

    public static final double DEAD_WHEEL_TICKS_PER_INCH = DEAD_WHEEL_TICKS_PER_ROTATION / DEAD_WHEEL_CIRCUMFERENCE_INCH;
    public static final double DEAD_WHEEL_TICKS_PER_MM = DEAD_WHEEL_TICKS_PER_ROTATION / DEAD_WHEEL_CIRCUMFERENCE;
    public static final double DEAD_WHEEL_MM_PER_TICK = DEAD_WHEEL_CIRCUMFERENCE / DEAD_WHEEL_TICKS_PER_ROTATION;

    public static final double MINIMUM_TIME_IN_SECONDS = 1e-3;
    public static final double CONVERT_TIME_TO_SECONDS = 1000;
    public static final double MINIMUM_DISTANCE_IN_TICKS = 1500;
    public static final double MINIMUM_DISTANCE_IN_INCHES = 0.5;

    public static final double TURN_TOLERANCE_IN_DEGREES = 4;
    public static final double TURN_TOLERANCE_IN_RADIANS = .035;

    public static final double HEIGHT_TOLERANCE = 200;

    public static  final  double ELBOW_EXTEND_SETTING = 0.7;

    public static final double CONVERT_METERS_TO_INCHES = 39.3701;

    public static final double DRIVE_RADIUS = WHEEL_BASE_WIDTH_MM * DEAD_WHEEL_TICKS_PER_MM;
    public static final double STRAFE_RADIUS = WHEEL_BASE_LENGTH_MM * DEAD_WHEEL_TICKS_PER_MM;

    public static final double APRIL_TAG_WEIGHT = 0.8;

//  public static final double TURN_SPEED = 0.2;

    public static final double MAX_KP = 0.80;

    public static final double MINIMUM_POWER_OUTPUT_DRIVE = 0.20;
    public static final double MINIMUM_POWER_OUTPUT_TURN = 0.2;

    public static final double MAX_INTEGRAL_XY = 20000;
    public static final double MAX_INTEGRAL_TURN = 100;

    public static final long MAX_AGE_NANOSECONDS = 250000000;

    private Constants() {
        throw new AssertionError("Cannot instantiate constants class");
    }
}

