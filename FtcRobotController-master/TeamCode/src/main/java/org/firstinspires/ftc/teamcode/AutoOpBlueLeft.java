package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoOpBlueLeft", group = "Main")
public class AutoOpBlueLeft extends LinearOpMode {

    RobotState initialState = new RobotState(-24, -63, 0, 0, 0, 0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.LEFT);

        waitForStart();

        robot.moveToPositionAndHeading(new RobotState(-5, -39, 0, 0, 0, 0, 2, 1));
        //robot.justArm(0.8,0.7,2,1);
        sleep(2000);
        robot.moveToPositionAndHeading(new RobotState(-48, -43, 0, 0, 0, 0, 2, 1));
        robot.justArm(0.5,0.7,1.5,1.5,true);
        robot.justArm(0,0.7,2,1,false);
        robot.wristUp();
        robot.moveToPositionAndHeading(new RobotState(-53, -53, -45, 0, 0, 0, 2, 1));
        robot.justArm(0.8,0.7,2,1,true);
        robot.wristDown();
        robot.moveToPositionAndHeading(new RobotState(-56, -42, 0, 0,0,0,2,1));
        robot.justArm(0,0.7,2,1,false);
        robot.wristUp();
        robot.moveToPositionAndHeading(new RobotState(-53, -53, -45, 0,0,0,2,1));
        robot.justArm(0.8,0.7,2,1,true);
        robot.wristDown();
        robot.moveToPositionAndHeading(new RobotState(-53, -53, -45, 0,0,0,2,1));
        robot.justArm(0,0.7,2,1,false);
    }
}
