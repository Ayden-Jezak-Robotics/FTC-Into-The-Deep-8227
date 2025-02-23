package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoOpSensorTest", group = "Test")
public class AutoOpSensorTest extends LinearOpMode {

    RobotState initialState = new RobotState(-24, -63, 0, 0, 0, false, false);

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.LEFT);

        waitForStart();

        robot.checkSensorReadings();
    }


}
