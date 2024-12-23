package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorUtility {

    private final HardwareMap hardwareMap;
    private final DcMotor backLeft, backRight, frontRight, frontLeft;

    MotorUtility(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.frontLeft = initializeMotor("frontLeft", DcMotorSimple.Direction.REVERSE);
        this.frontRight = initializeMotor("frontRight", DcMotorSimple.Direction.FORWARD);
        this.backLeft = initializeMotor("backLeft", DcMotorSimple.Direction.REVERSE);
        this.backRight = initializeMotor("backRight", DcMotorSimple.Direction.FORWARD);
    }

    DcMotor initializeMotor(String name, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        if (motor == null) {
            telemetry.addData("Error", "Motor not found: " + name);
            telemetry.update();
        } else {
            motor.setDirection(direction);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
        return motor;
    }

    void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

}
