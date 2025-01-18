package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class IMUUtility {

    private final BHI260IMU imu;
    private double previousHeading;

    IMUUtility(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BHI260IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        IMU.Parameters imuParameters = new IMU.Parameters(orientationOnRobot);

        imu.initialize(imuParameters);

        previousHeading = 0;
    }

    double getCurrentHeading() {
        double currentHeadingInRadians;

        currentHeadingInRadians = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX, // TODO - Is this the right order to get firstAngle?
                AngleUnit.RADIANS
        ).firstAngle;

        return (currentHeadingInRadians + (2 * Math.PI)) % (2 * Math.PI);
    }

    public double getPreviousHeading() {
        return this.previousHeading;
    }

    public void setPreviousHeading(double newValue) {
        this.previousHeading = newValue;
    }

    void resetIMU() {
        imu.resetYaw();
    }
}
