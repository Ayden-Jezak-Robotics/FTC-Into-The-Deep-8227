package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmUtility {

    private final HardwareMap hardwareMap;
    private final DcMotor armLeft, armRight;

    public ArmUtility(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.armLeft = initializeMotor("armLeft", DcMotorSimple.Direction.FORWARD);
        this.armRight = initializeMotor("armRight", DcMotorSimple.Direction.REVERSE);
    }

    DcMotor initializeMotor(String name, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);

        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return motor;
    }
}
