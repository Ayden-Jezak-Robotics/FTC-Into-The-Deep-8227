package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.Iterator;


public class VisionUtility {

    private final AprilTagProcessor myAprilTagProcessor;
    private final VisionPortal myVisionPortal;
    private final HardwareMap myHardwareMap;

    VisionUtility(HardwareMap hardwareMap) {

        Position cameraPosition = new Position(DistanceUnit.MM, 0, 0, 0, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);

        this.myHardwareMap = hardwareMap;

        this.myAprilTagProcessor = new AprilTagProcessor.Builder()
               .setCameraPose(cameraPosition, cameraOrientation)
               .setDrawAxes(true)
               //Need to calibrate and find
               /*.setCameraIntrinsicMatrix(fx, fy, cx, cy) // Intrinsic matrix for camera calibration
               .setLensDistortionCoefficients(k1, k2, p1, p2, k3) // Distortion coefficients*/
               .build();

        this.myVisionPortal = new VisionPortal.Builder()
               .setShowStatsOverlay(true)
               .setCamera(myHardwareMap.get(WebcamName.class, "Webcam 1"))
               .addProcessor(myAprilTagProcessor)
               .setCameraResolution(new Size(640, 480))
               .setStreamFormat(VisionPortal.StreamFormat.YUY2)
               .enableLiveView(true)
               .build();
    }

    private List<AprilTagDetection> getDetections() {
        long currentTime = System.nanoTime();

        List<AprilTagDetection> myAprilTagDetections = myAprilTagProcessor.getDetections();

        if (myAprilTagDetections == null || myAprilTagDetections.isEmpty()) {
            // No detections found; return null or handle appropriately
            return null;
        }

        //NEW for-each loop is only used for iterating, and runs into an error if we try to modify the list
        Iterator<AprilTagDetection> iterator = myAprilTagDetections.iterator();
        while (iterator.hasNext()) {
            AprilTagDetection detection = iterator.next();
            if (detection.metadata == null || detection.robotPose == null ||
                currentTime - detection.frameAcquisitionNanoTime >= Constants.MAX_AGE_NANOSECONDS) {
                iterator.remove(); // Remove invalid detections
            }
        }
        /*
        for (AprilTagDetection detection : myAprilTagDetections) {
            if (detection.metadata != null && detection.robotPose != null) {

                long detectionAge = currentTime - detection.frameAcquisitionNanoTime;
                if (detectionAge >= Constants.MAX_AGE_NANOSECONDS) {
                    myAprilTagDetections.remove(detection);

                }
            }
            else {
                myAprilTagDetections.remove(detection);
            }
        }*/

        return myAprilTagDetections;
    }

    public Pose3D getPose() {
        Pose3D robotPosition = null; // Start with a null position to handle cases where no detection is valid

        List<AprilTagDetection> myAprilTagDetections = getDetections();

        if (myAprilTagDetections != null) {
            for (AprilTagDetection detection : myAprilTagDetections) {
                if (detection != null && detection.robotPose != null) //NEW sometimes detections are valid and robotPoses if the tag is barely detected at a far distance
                {
                    robotPosition = detection.robotPose;
                    break; // Assuming you want the position from the first valid detection
                }
            }
        }

        return robotPosition; // Could still be null if no valid detection found
    }

}
