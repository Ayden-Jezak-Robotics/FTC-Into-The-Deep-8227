package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoOpSquare", group = "Test")
public class AutoOpSquare extends LinearOpMode {

    RobotState initialState = new RobotState(0, 0, 0, 0,0,false,false);

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.LEFT);

        int sleepTime = 1000;

        waitForStart();


        robot.moveToNewRobotState(new RobotState(0, 24, 0, 0,0,false,false));
        sleep(sleepTime);
        robot.moveToNewRobotState(new RobotState(24, 24, 0, 0,0,false,false));
        sleep(sleepTime);
        robot.moveToNewRobotState(new RobotState(24, 0, 0, 0,0,false,false));
        sleep(sleepTime);
        robot.moveToNewRobotState(new RobotState(0, 0, 0, 0,0,false,false));

    }
}
