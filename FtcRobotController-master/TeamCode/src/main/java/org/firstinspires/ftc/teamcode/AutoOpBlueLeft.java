package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoOpBlueLeft", group = "Draft")
public class AutoOpBlueLeft extends LinearOpMode {

    RobotState initialState = new RobotState(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.LEFT);

        waitForStart();

        //robot.turnOn();
        robot.moveToPositionAndHeading(new RobotState(24, 24, 0));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(-12, 20, 0));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(-24, 6, -45));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(-22, 20, 0));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(-24, 6, -45));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(-26, 20, 30));
        sleep(1000);
        robot.moveToPositionAndHeading(new RobotState(-24, 6, -45));

    }


}
