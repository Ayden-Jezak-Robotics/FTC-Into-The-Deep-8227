package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class IMUUtility {

    private BNO055IMU imu;
    private double previousHeading = 0;

    IMUUtility(HardwareMap hardwareMap, Telemetry telemetry) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }

    boolean calibrateIMU() {
        return imu.isGyroCalibrated();
    }

    double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    Position getIMUPosition() {
        return  imu.getPosition();
    }

    public double getPreviousHeading()
    {
        return this.previousHeading;
    }

    public void setPreviousHeading(double previousHeading) {
        this.previousHeading = previousHeading;
    }

    public double normalizeHeading(double angle) {
        while (angle >= 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

}
