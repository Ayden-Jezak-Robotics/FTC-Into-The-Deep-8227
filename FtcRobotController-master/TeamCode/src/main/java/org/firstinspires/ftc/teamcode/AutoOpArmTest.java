package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoOpArmTest", group = "Test")
public class AutoOpArmTest extends LinearOpMode {

    RobotState initialState = new RobotState(0, 0, 0, 0, 0, false, false, false);

    /// In Degrees

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.RIGHT);

        waitForStart();

        robot.moveToNewRobotState(new RobotState(0, 0, 0, 0, 0, true, false, false));
        sleep(1000);
        robot.moveToNewRobotState(new RobotState(0, 0, 0, 4000, 0, true, true, false));
        sleep(1000);
        robot.moveToNewRobotState(new RobotState(0, 0, 0, 1000, 0, true, false, true));
        sleep(1000);
        robot.moveToNewRobotState(new RobotState(0, 0, 0, 1000, .5, true, false, false));
        sleep(1000);
        robot.moveToNewRobotState(new RobotState(0, 0, 0, 0, 0, true, true, false));

    }
}