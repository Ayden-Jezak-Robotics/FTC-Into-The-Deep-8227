package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DeadWheelUtility {
    private final HardwareMap hardwareMap;
    private final DcMotor rightDeadWheel, leftDeadWheel, centerDeadWheel;
    private int previousLeft, previousRight, previousCenter;

    DeadWheelUtility(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.leftDeadWheel = initializeDeadWheel("frontLeft", DcMotorSimple.Direction.FORWARD);
        this.rightDeadWheel = initializeDeadWheel("rightDeadWheel", DcMotorSimple.Direction.REVERSE);
        this.centerDeadWheel = initializeDeadWheel("centerDeadWheel", DcMotorSimple.Direction.FORWARD);
        previousLeft = 0;
        previousRight = 0;
        previousCenter = 0;
    }

    DcMotor initializeDeadWheel(String name, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    int getCurrentValue(DeadWheel type) {

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

    int getPreviousValue(DeadWheel type) {

        switch (type) {
            case CENTER:
                return previousCenter;
            case LEFT:
                return  previousLeft;
            case RIGHT:
                return  previousRight;
            default:
                return 0;
        }
    }

    void setPreviousLeft(int newValue) {
        previousLeft = newValue;
    }

    void setPreviousRight(int newValue) {
        previousRight = newValue;
    }

    void setPreviousCenter(int newValue) {
        previousCenter = newValue;
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
