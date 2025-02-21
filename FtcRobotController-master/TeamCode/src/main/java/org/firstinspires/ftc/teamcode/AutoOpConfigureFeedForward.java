package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoOpConfigureFeedForward", group = "Test")
public class AutoOpConfigureFeedForward extends LinearOpMode {

    RobotState initialState = new RobotState(0, 0, 0, 0, 0, false, false, false);

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.LEFT);

        waitForStart();

        robot.moveToNewRobotState(new RobotState(0, 24, 0, 0, 0, false, false, false));

        sleep(2000);

        robot.moveToNewRobotState(new RobotState(24, 24, 0, 0, 0, false, false, false));

    }


}
