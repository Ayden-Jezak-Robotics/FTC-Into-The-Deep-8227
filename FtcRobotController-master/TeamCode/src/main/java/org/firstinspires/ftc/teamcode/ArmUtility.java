package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ArmUtility {

    private final Servo leftArmServo;
    private final Servo rightArmServo;
    private final Servo grabberServo;
    private final DcMotor armLeft;
    private final DcMotor armRight;

    public ArmUtility(HardwareMap hardwareMap) {
        this.leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        this.rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        this.grabberServo = hardwareMap.get(Servo.class, "grabberServo");

        this.armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        this.armRight = hardwareMap.get(DcMotor.class, "armRight");

        // Initialize servo positions
        leftArmServo.setPosition(0.0);
        rightArmServo.setPosition(0.0);
        grabberServo.setPosition(0.5); // Assume 0.5 is the neutral position for grabber

        // Initialize motor behavior
        armRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //Is armLeft supposed to be forward? QUESTION

        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void grabObject() {
        // Open grabber to pick up object
        grabberServo.setPosition(0.4); // Assume 0.4 is open position, Probably have to adjust this later

        // Lower arms to floor position
        armLeft.setTargetPosition(-1100); // Move down to grab position, Definitely have to adjust
        armRight.setTargetPosition(-1100);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLeft.setPower(0.5);
        armRight.setPower(0.5);

        while (armLeft.isBusy() && armRight.isBusy()) {
            // Wait for arms to reach position
        }

        // Move wrists forward
        leftArmServo.setPosition(0.1);//probably have to adjust
        rightArmServo.setPosition(0.1); //adjust

        // Close grabber to hold object
        grabberServo.setPosition(0.0); // Assume 0.0 is closed position
      
        leftArmServo.setPosition(-0.1);//probably have to adjust
        rightArmServo.setPosition(-0.1); //adjust
    }

    public void putInBasket() {
        // Raise arms to basket position
        armLeft.setTargetPosition(5500); // Move up to basket position and adjust
        armRight.setTargetPosition(5500); //adjust
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLeft.setPower(0.75);
        armRight.setPower(0.75);

        while (armLeft.isBusy() && armRight.isBusy()) {
            // Wait for arms to reach position
        }

        leftArmServo.setPosition(0.1);//probably have to adjust
        rightArmServo.setPosition(0.1); //adjust

        grabberServo.setPosition(0.4); // Adjust to open position
        // Reset wrists to neutral position
        leftArmServo.setPosition(0.5);
        rightArmServo.setPosition(0.5);
    }

    public void resetArms() {
        // Reset arms to default position
        armLeft.setTargetPosition(0);
        armRight.setTargetPosition(0);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLeft.setPower(0.5);
        armRight.setPower(0.5);

        while (armLeft.isBusy() && armRight.isBusy()) {
            // Wait for arms to reach position
        }

        // Reset wrists to default position
        leftArmServo.setPosition(0.0);
        rightArmServo.setPosition(0.0);

        // Reset grabber
        grabberServo.setPosition(0.5);
    }
}