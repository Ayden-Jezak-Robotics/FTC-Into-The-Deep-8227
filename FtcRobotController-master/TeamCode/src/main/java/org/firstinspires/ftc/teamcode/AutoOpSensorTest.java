package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoOpSensorTest", group = "Draft")
public class AutoOpSensorTest extends LinearOpMode {

    RobotState initialState = new RobotState(0, 0, 0, 0,0,0);

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.LEFT);

        waitForStart();

        //robot.checkSensorReadings();
        //robot.armUp(2000);
        robot.extendWithTime(1.0,1.0,2,1,2);
        robot.extendWithTime(0.0,1.0,6,2,6);
        //robot.handleArmWithTime(1,1,2,1);
        //robot.handleArmWithTime(0,1,6,0);
    }


}
