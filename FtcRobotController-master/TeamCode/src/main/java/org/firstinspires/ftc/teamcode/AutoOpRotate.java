package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoOpRotate", group = "Draft")
public class AutoOpRotate extends LinearOpMode {

    RobotState initialState = new RobotState(0, 0, 0);

    /// In Degrees

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.RIGHT);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (timer.seconds() <= 25 ) {

            robot.turnOn(0.4);
        }
        

    }
}