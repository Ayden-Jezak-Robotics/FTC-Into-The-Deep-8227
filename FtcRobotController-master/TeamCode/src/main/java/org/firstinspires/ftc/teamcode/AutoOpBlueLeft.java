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

        robot.moveToNewRobotState(new RobotState(-5, -39, 0, 0, 0, true, false, false));
        //robot.justArm(0.8,0.7,2,1);
        sleep(2000);
        robot.moveToNewRobotState(new RobotState(-48, -43, 0, 0, 0, true, false, true));
        robot.justArm(0.5,0.7,1.5,1.5,true);
        robot.justArm(0,0.7,2,1,false);
        robot.wristUp();
        robot.moveToNewRobotState(new RobotState(-53, -53, -45, 0, 0, 0, 2, 1));
        robot.justArm(0.8,0.7,2,1,true);
        robot.wristDown();
        robot.moveToNewRobotState(new RobotState(-56, -42, 0, 0,0,0,2,1));
        robot.justArm(0,0.7,2,1,false);
        robot.wristUp();
        robot.moveToNewRobotState(new RobotState(-53, -53, -45, 0,0,0,2,1));
        robot.justArm(0.8,0.7,2,1,true);
        robot.wristDown();
        robot.moveToNewRobotState(new RobotState(-53, -53, -45, 0,0,0,2,1));
        robot.justArm(0,0.7,2,1,false);
    }
}
