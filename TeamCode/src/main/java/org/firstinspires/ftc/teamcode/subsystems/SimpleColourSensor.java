package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.configs.Globals.simpleTelemetry;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.configs.BotHardware;
import org.firstinspires.ftc.teamcode.configs.Globals;
import org.firstinspires.ftc.teamcode.utils.SubsystemBase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@Configurable
public class SimpleColourSensor implements SubsystemBase {

    private PredominantColorProcessor colorSensor;
    private PredominantColorProcessor.Result result;

    @Override
    public void init(BotHardware botHardware) {
        colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE
                )
                .build();

        // 2. Attach the processor to VisionPortal
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(botHardware.insideCam)
                .addProcessor(colorSensor)
                .build();

        PanelsCameraStream.INSTANCE.startStream(visionPortal, 15);
    }

    @Override
    public void update() {
        result = colorSensor.getAnalysis();
        simpleTelemetry.addData("CamColourSensor", "Colour Classified: " + result.closestSwatch.toString());
    }

    public Globals.BallType getBallType() {
        switch (result.closestSwatch) {
            case RED:
            case BLUE:
                return Globals.BallType.Big;
            case YELLOW:
                return Globals.BallType.Small;
            default:
                return Globals.BallType.None;
        }

    }
}
