package org.firstinspires.ftc.teamcode.Robot.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pedrio.Vision.OpenCv.CSVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

public class OpenCvTest extends OpMode {
    private CSVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    @Override
    public void init() {
        visionProcessor = new CSVisionProcessor(125, 0, 0, 150, 350, 400, 350);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);//this would be in the hardware class

    }

    @Override
    public void loop() {
        telemetry.addData("Identified", visionProcessor.getStartingPosition());
        telemetry.update();

    }
}
