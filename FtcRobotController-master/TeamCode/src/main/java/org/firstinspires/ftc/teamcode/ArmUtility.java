package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmUtility {

    private final HardwareMap hardwareMap;
    private final DcMotor leftArmMotor, rightArmMotor;

    private final Servo leftArmServo, rightArmServo;
    private final Servo wristServo, grabberServo, elbowServo;

    private int previousArm;
    private double previousRightAngle, previousLeftAngle;

    public ArmUtility(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        this.leftArmMotor = initializeMotor("armLeft", DcMotor.Direction.FORWARD);
        this.rightArmMotor = initializeMotor("armRight", DcMotor.Direction.REVERSE);

        this.leftArmServo = initializeServo("leftArmServo");
        this.rightArmServo = initializeServo("rightArmServo");
        this.wristServo = initializeServo("wristServo");
        this.grabberServo = initializeServo("grabberServo");
        this.elbowServo = initializeServo("elbowServo");

        previousArm = 0;
    }

    DcMotor initializeMotor(String name, DcMotor.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    Servo initializeServo(String name) {
        Servo servo = hardwareMap.get(Servo.class, name);
        return servo;
    }

    void angleArm() {
        rightArmServo.setPosition(0.8);
        leftArmServo.setPosition(0.75);
        sleep(200);
        rightArmServo.setPosition(0.7);
        leftArmServo.setPosition(0.85);
        sleep(200);
        rightArmServo.setPosition(0.65);
        leftArmServo.setPosition(0.9);
        sleep(200);
        rightArmServo.setPosition(0.6);
        leftArmServo.setPosition(0.95);
    }

    void angleArmTo(double position)
    {
        rightArmServo.setPosition(0.9 - position);
        leftArmServo.setPosition(0.65 + position);
    }

    void angleArmToBase() {
        rightArmServo.setPosition(0.9); //BASE is when it's facing up
        leftArmServo.setPosition(0.65);
    }

    void extendElbow(double position)
    {
        elbowServo.setPosition(position)
    }

    void openGrabber() {
        grabberServo.setPosition(0.6);
    }

    void closeGrabber() {
        grabberServo.setPosition(0.0);
    }

    void setWristPosition(double position){
        wristServo.setPosition(position);
    }

    void setArmPowers(double armPower) {
        
        leftArmMotor.setPower(armPower);
        rightArmMotor.setPower(armPower);
    }

    int getPreviousArm(DeadWheel type) {
        return previousArm;
    }

    void setPreviousArm(int newValue) {
        previousArm = newValue;
    }

    void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
