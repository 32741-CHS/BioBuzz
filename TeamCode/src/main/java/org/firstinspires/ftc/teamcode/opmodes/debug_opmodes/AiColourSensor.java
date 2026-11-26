package org.firstinspires.ftc.teamcode.opmodes.debug_opmodes;

import com.bylazar.camerastream.PanelsCameraStream;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@TeleOp(name = "CamColourSensor Example", group = "Tests")
public class AiColourSensor extends LinearOpMode {
    @Override
    public void runOpMode() {
        // 1. Set up the processor with a small ROI over the intake
        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE) // Empty intake background
                .build();

        // 2. Attach the processor to VisionPortal
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(colorSensor)
                .build();

        PanelsCameraStream.INSTANCE.startStream(visionPortal, 15);

        waitForStart();

        while (opModeIsActive()) {
            PredominantColorProcessor.Result result = colorSensor.getAnalysis();

            // 3. Act on the detected colour
            telemetry.addData("CamColourSensor", "Colour Classified: " + result.closestSwatch.toString());
        }
        visionPortal.close();
    }
}
