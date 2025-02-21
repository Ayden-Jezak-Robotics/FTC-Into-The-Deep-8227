package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotState {

    final Position position;
    final double heading; // In Degrees
    final double armHeight;
    final double armAngle;
    final double extend;
    final double armAngleTime;
    final double extendTime;


    public RobotState(int x, int y, int heading, double armHeight, double armAngle, double extend, double armAngleTime, double extendTime) {
        this.position = new Position(DistanceUnit.INCH, x, y, 0, System.nanoTime());
        this.heading = heading; // in degrees
        this.armHeight = armHeight;
        this.armAngle = armAngle;
        this.extend = extend;
        this.armAngleTime = armAngleTime;
        this.extendTime = extendTime;
    }
}
