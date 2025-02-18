package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoOpBlueRight", group = "Draft")
public class AutoOpRedRight extends LinearOpMode {

    RobotState initialState = new RobotState(0, 0, 0, 0,0,0,0,0);

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.LEFT);

        waitForStart();
        /*
        robot.moveToPositionAndHeading(new RobotState(-32, 26, 0, 0,0,0));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(-32, 6, 0, 0,0,0));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(12, 48, 0, 0,0,0));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(12, 8, 0, 0,0,0));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(12, 48, 0, 0,0,0));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(20, 48, 0, 0,0,0));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(20, 8, 0, 0,0,0));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(20, 48, 0, 0,0,0));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(26, 48, 0, 0,0,0));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(26, 8, 0, 0,0,0));

         */
    }


}
