package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Autonomous(name = "AutoOpConfigureFeedForward", group = "Draft")
public class AutoOpConfigureFeedForward extends LinearOpMode {

    Position initialPosition = new Position(DistanceUnit.INCH, 0, 0, 0, System.nanoTime());
    double initialHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        
        Robot robot = new Robot(this, hardwareMap, telemetry, initialPosition, initialHeading);

        waitForStart();

        robot.moveToPositionAndHeading(new Position(DistanceUnit.INCH, 0, 24, 0, 0), 0);

        sleep(2000);

        robot.moveToPositionAndHeading(new Position(DistanceUnit.INCH, 24, 24, 0, 0), 0);

    }


}
