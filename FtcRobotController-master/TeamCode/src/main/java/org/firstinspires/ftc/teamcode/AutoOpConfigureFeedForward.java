package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoOpConfigureFeedForward", group = "Test")
public class AutoOpConfigureFeedForward extends LinearOpMode {

    RobotState initialState = new RobotState(0, 0, 0, 0, 0, 0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.LEFT);

        waitForStart();

        robot.moveToPositionAndHeading(new RobotState(0, 24, 0, 0, 0, 0, 0, 0));

        sleep(2000);

        robot.moveToPositionAndHeading(new RobotState(24, 24, 0, 0, 0, 0, 0, 0));

    }


}
