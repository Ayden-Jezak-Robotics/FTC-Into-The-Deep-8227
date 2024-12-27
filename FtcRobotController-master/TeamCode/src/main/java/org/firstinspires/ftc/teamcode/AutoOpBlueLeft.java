package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Autonomous(name = "AutoOpBlueLeft", group = "Draft")
public class AutoOpBlueLeft extends LinearOpMode {

    Position initialPosition = new Position(DistanceUnit.INCH, 24, 64, 0, System.nanoTime());
    double initialHeading = -90;

    @Override
    public void runOpMode() throws InterruptedException {

        final Robot robot = new Robot(this, hardwareMap, telemetry, initialPosition, initialHeading);

        waitForStart();

        robot.moveToPositionAndHeading(new Position(DistanceUnit.INCH, 48, 24, 0, 0), -90);

        //robot.moveToPositionAndHeading(new Position(DistanceUnit.INCH, 24, -12, 0, 0), 90);

    }


}
