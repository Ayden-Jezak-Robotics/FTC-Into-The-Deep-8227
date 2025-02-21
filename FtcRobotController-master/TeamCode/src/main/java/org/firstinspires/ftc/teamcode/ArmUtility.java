package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmUtility {

    private final HardwareMap hardwareMap;
    private final DcMotor leftArmMotor, rightArmMotor;

    private final Servo leftShoulderServo, rightShoulderServo;
    private final Servo elbowServo, grabberServo, wristServo;

    private int previousArm;
    private double previousRightAngle, previousLeftAngle;

    public ArmUtility(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        this.leftArmMotor = initializeArmMotor("armLeft", DcMotor.Direction.FORWARD);
        this.rightArmMotor = initializeArmMotor("armRight", DcMotor.Direction.REVERSE);

        this.leftShoulderServo = initializeServo("leftArmServo",1);
        this.rightShoulderServo = initializeServo("rightArmServo",0);
        this.elbowServo = initializeServo("elbowServo",0);
        this.wristServo = initializeServo("wristServo",0.1);
        this.grabberServo = initializeServo("grabberServo",0.3);

        previousArm = 0;
    }

    DcMotor initializeArmMotor(String name, DcMotor.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    Servo initializeServo(String name, double position) {
        Servo servo = hardwareMap.get(Servo.class, name);
        servo.setPosition(position);
        return servo;
    }

    int getAverageCurrentPosition()
    {
        int average = (leftArmMotor.getCurrentPosition() + rightArmMotor.getCurrentPosition())/2;
        return average;
    }

    double getShoulderPosition()
    {
        return rightShoulderServo.getPosition();
    }

    double getElbowPosition()
    {
       return elbowServo.getPosition();
    }

    void angleArm() {
        rightShoulderServo.setPosition(0.8);
        leftShoulderServo.setPosition(0.75);
        sleep(200);
        rightShoulderServo.setPosition(0.7);
        leftShoulderServo.setPosition(0.85);
        sleep(200);
        rightShoulderServo.setPosition(0.65);
        leftShoulderServo.setPosition(0.9);
        sleep(200);
        rightShoulderServo.setPosition(0.6);
        leftShoulderServo.setPosition(0.95);
    }

    void rotateArmToAngle(double position)
    {
        rightShoulderServo.setPosition(position);
        leftShoulderServo.setPosition(1-position);
    }

    void angleArmToBase() {
        rightShoulderServo.setPosition(0.9); //BASE is when it's facing up
        leftShoulderServo.setPosition(0.65);
    }

    void extendElbow(double position)
    {
        elbowServo.setPosition(position);
    }

    void openGrabber() {
        grabberServo.setPosition(0.7);
//        sleep(500);
    }

    void closeGrabber() {
        grabberServo.setPosition(0.3);
//        sleep(500);

    }


    void setWristPosition(double position){
        wristServo.setPosition(position);
    }

    void wristDown()
    {
        wristServo.setPosition(0.1);
    }

    void wristUp()
    {
        wristServo.setPosition(0.8);
    }


    void setArmPowers(double armPower) {
        leftArmMotor.setPower(armPower);
        rightArmMotor.setPower(armPower);
    }

    void setHoldingPower()
    {
        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    int getPreviousArm() {
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

    public void stopMotors() {
        leftArmMotor.setPower(0);
        rightArmMotor.setPower(0);
    }
}
