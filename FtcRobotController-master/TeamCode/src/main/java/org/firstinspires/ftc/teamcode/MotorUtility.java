package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorUtility {

    private final HardwareMap hardwareMap;
    private final DcMotor backLeft, backRight, frontRight, frontLeft;

    MotorUtility(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.frontLeft = initializeMotor("frontLeft", DcMotorSimple.Direction.REVERSE);
        this.frontRight = initializeMotor("frontRight", DcMotorSimple.Direction.FORWARD);
        this.backLeft = initializeMotor("backLeft", DcMotorSimple.Direction.REVERSE);
        this.backRight = initializeMotor("backRight", DcMotorSimple.Direction.FORWARD);
    }

    DcMotor initializeMotor(String name, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);

        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return motor;
    }

    void setMotorPowers(double xPower, double yPower, double turnPower) {

        double frontLeftPower = yPower + xPower - turnPower;
        double frontRightPower = yPower - xPower + turnPower;
        double backLeftPower = yPower - xPower - turnPower;
        double backRightPower = yPower + xPower + turnPower;

        // Normalize powers if they exceed 1
        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

}
