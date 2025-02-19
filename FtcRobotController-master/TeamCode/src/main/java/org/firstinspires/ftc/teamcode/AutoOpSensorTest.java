package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoOpSensorTest", group = "Test")
public class AutoOpSensorTest extends LinearOpMode {

    RobotState initialState = new RobotState(-24, -63, 0, 0, 0, 0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.LEFT);

        waitForStart();

        //robot.moveToPositionAndHeading();

        //robot.checkSensorReadings();
        //robot.armUp(2000);
        //robot.extendWithTime(1.0,1.0,2,1,2);
        //robot.extendWithTime(0.0,1.0,6,2,6);
        //robot.handleArmWithTime(0.8,0.7,2,1);
        //robot.handleArmWithTime(0,0.7,6,0);
        //robot.turnOnArm();
        //robot.lookArm();
        robot.justArm(0.8,0.7,2,1,false);
        robot.justArm(0,0.7,4,0,false);
        //robot.pickUpObject();
    }


}
