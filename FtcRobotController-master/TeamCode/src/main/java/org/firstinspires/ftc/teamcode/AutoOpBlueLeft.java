package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoOpBlueLeft", group = "Main")
public class AutoOpBlueLeft extends LinearOpMode {

    RobotState initialState = new RobotState(-24, -63, 0, 0, 0, false, false, false);

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.LEFT);

        waitForStart();

        robot.moveToNewRobotState(new RobotState(-5, -39, 0, 0, 0, false, false, false));
        sleep(2000);
        robot.moveToNewRobotState(new RobotState(-48, -43, 0, 0, 0.5, true, false, true));
        robot.moveToNewRobotState(new RobotState(-48, -43, 0, 0, 0.0, true, false, false));
        robot.moveToNewRobotState(new RobotState(-53, -53, -45, 0, 0.8, true, true, false));
        robot.moveToNewRobotState(new RobotState(-53, -53, -45, 0, 0.8, true, true, true));
        robot.moveToNewRobotState(new RobotState(-56, -42, 0, 0,0,true,false,true));
        robot.moveToNewRobotState(new RobotState(-56, -42, 0, 0,0,true,false,false));
        robot.moveToNewRobotState(new RobotState(-53, -53, -45, 0,0.8,true,true,false));
        robot.moveToNewRobotState(new RobotState(-53, -53, -45, 0,0.8,true,true,true));
        robot.moveToNewRobotState(new RobotState(-20, -20, 90, 0,0,true,true,false));

    }
}
