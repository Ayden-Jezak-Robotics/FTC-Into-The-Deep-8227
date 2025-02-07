package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DeadWheelUtility {
    private final HardwareMap hardwareMap;
    private final DcMotor deadWheelDrive, deadWheelStrafe;
    private int previousDrive, previousStrafe, previousArm;

    DeadWheelUtility(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        // TODO: Rename DeadWheels in Drive Hub
        this.deadWheelDrive = initializeDeadWheel("driveDeadWheel", DcMotorSimple.Direction.REVERSE);
        this.deadWheelStrafe = initializeDeadWheel("strafeDeadWheel", DcMotorSimple.Direction.REVERSE);
        previousDrive = 0;
        previousStrafe = 0;
    }

    DcMotor initializeDeadWheel(String name, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    int getCurrentValue(DeadWheel type) {

        switch (type) {
            case DRIVE:
                return deadWheelDrive.getCurrentPosition();
            case STRAFE:
                return deadWheelStrafe.getCurrentPosition();
            default:
                return 0;
        }
    }

    int getPreviousValue(DeadWheel type) {

        switch (type) {
            case DRIVE:
                return previousDrive;
            case STRAFE:
                return previousStrafe;
            default:
                return 0;
        }
    }

    void setPreviousDrive(int newValue) {
        previousDrive = newValue;
    }

    void setPreviousStrafe(int newValue) {
        previousStrafe = newValue;
    }


    void resetEncoders() {
        deadWheelDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        previousDrive = 0;

        deadWheelStrafe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        previousStrafe = 0;
    }
}
