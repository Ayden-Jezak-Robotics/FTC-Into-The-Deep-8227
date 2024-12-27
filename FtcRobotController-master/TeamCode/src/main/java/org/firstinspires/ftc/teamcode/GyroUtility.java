package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GyroUtility {

    private BNO055IMU imu;

    GyroUtility(HardwareMap hardwareMap) throws InterruptedException {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        while (!calibrateIMU()) {
            sleep(100);
        }
    }

    boolean calibrateIMU() {
        return imu.isGyroCalibrated();
    }

    double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public double normalizeHeading(double angle) {
        while (angle >= 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

//    public double normalizeAngle(double angle) {
//        double normalizedAngle;
//        if (angle > 180) {
//            normalizedAngle = angle - 360;
//        } else if (angle < -180) {
//            normalizedAngle = angle + 360;
//        } else {
//            normalizedAngle = angle;
//        }
//        return normalizedAngle;
//    }
}
