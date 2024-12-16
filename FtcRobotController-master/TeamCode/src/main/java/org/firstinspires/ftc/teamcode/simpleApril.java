

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;


import java.util.List;
@TeleOp(name = "simpleApril", group = "Concept")
public class simpleApril extends LinearOpMode {

    public void runOpMode() {
        VisionPortal myVisionPortal = null; // Declare outside the try block

        try {
            myVisionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"),
                    null // No processor, just raw feed
            );
            telemetry.addData("Camera", "Initialized successfully");
        } catch (Exception e) {
            telemetry.addData("Camera Error", e.getMessage());
        }

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running...");
            telemetry.update();
        }

        // Close the VisionPortal when done to free resources
        if (myVisionPortal != null) {
            myVisionPortal.close();
        }
    }
}
