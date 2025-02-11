package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "MainOP", group = "Draft")
public class ServoTest extends LinearOpMode{
    private Servo myServo;

    @Override
    public void runOpMode() throws InterruptedException {

        myServo = hardwareMap.get(Servo.class, "wristServo"); // Replace "servoName" with actual servo name

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        myServo.setPosition(0.5); //middle

        waitForStart();

        //myServo.setPosition(1.0); // Move to max position

        double target = 1.0;
        double targetTime = 10;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive()) {
            double currentTime = timer.seconds();
            double ratio = currentTime/targetTime;
            double pos = target*ratio;
            myServo.setPosition(pos);
            telemetry.addData("Servo Position", myServo.getPosition());
            telemetry.update();
        }
    }
}