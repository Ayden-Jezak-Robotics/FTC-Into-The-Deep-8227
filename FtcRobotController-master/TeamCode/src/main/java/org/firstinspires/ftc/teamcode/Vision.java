package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import android.util.Size;

@TeleOp(name = "Vision", group = "Concept")
public class Vision extends LinearOpMode
{

    public void runOpMode() throws InterruptedException
    {
//        int[] portalsList = VisionPortal.makeMultiPortalView(1 /*or however many april tags you have */, VisionPortal.MultiPortalLayout.HORIZONTAL);

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();

        waitForStart();

        while (!isStopRequested() && opModeIsActive())
        {

            if (!tagProcessor.getDetections().isEmpty())
            {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addData("x", tag.robotPose.getPosition().x);
                telemetry.addData("y", tag.robotPose.getPosition().y);
                telemetry.addData("z", tag.ftcPose.z);

                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
                telemetry.addData("Heading", tag.ftcPose.yaw);
            } else {
                telemetry.addData("Error", "No April tag in view");
            }
            telemetry.update();
        }

    }
}