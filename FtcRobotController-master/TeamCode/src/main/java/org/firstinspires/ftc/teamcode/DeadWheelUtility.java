package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DeadWheelUtility {
    private final HardwareMap hardwareMap;
    private final DcMotor rightDeadWheel, leftDeadWheel, centerDeadWheel;
    private double previousLeft = 0, previousRight = 0, previousCenter = 0;

    DeadWheelUtility(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.leftDeadWheel = initializeDeadWheel("leftDeadWheel", DcMotorSimple.Direction.REVERSE);
        this.rightDeadWheel = initializeDeadWheel("rightDeadWheel", DcMotorSimple.Direction.FORWARD);
        this.centerDeadWheel = initializeDeadWheel("centerDeadWheel", DcMotorSimple.Direction.REVERSE);
    }

    DcMotor initializeDeadWheel(String name, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    int getPosition(DeadWheel type) {

        switch (type) {
            case CENTER:
                return centerDeadWheel.getCurrentPosition();
            case LEFT:
                return  leftDeadWheel.getCurrentPosition();
            case RIGHT:
                return  rightDeadWheel.getCurrentPosition();
            default:
                return 0;
        }
    }

    void setPreviousLeft(double newValue) {
        previousLeft = newValue;
    }

    public double getPreviousLeft() {
        return previousLeft;
    }

    void setPreviousRight(double newValue) {
        previousRight = newValue;
    }

    public double getPreviousRight() {
        return previousRight;
    }

    void setPreviousCenter(double newValue) {
        previousCenter = newValue;
    }

    public double getPreviousCenter() {
        return previousCenter;
    }

    void resetEncoders() {
        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        previousLeft = 0;
        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        previousRight = 0;
        centerDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        previousCenter = 0;
    }
}
