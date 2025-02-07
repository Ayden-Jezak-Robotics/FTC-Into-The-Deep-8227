package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoOpBlueLeft", group = "Draft")
public class AutoOpBlueLeft extends LinearOpMode {

    RobotState initialState = new RobotState(0, 0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.LEFT);

        waitForStart();

        robot.moveToPositionAndHeading(new RobotState(32, 26, 0, 0));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(-9, 20, 0, 0));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(-18, 4, -45, 0));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(-18, 20, 0, 0));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(-18, 4, -45, 0));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(-20, 20, 30, 0));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(-18, 4, -45, 0));

    }


}
