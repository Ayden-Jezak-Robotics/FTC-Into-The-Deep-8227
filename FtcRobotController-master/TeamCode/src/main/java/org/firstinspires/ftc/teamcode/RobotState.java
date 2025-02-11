package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotState {

    final Position position;
    final double heading; // In Degrees
    final double height;
    final double angle;
    final double extend;


    // Constructor to initialize the state
    public RobotState(int x, int y, int heading, double height, double angle, double extend) {
        this.position = new Position(DistanceUnit.INCH, x, y, 0, System.nanoTime());
        this.heading = heading; // in degrees
        this.height = height;
        this.angle = angle;
        this.extend = extend;
    }

}
