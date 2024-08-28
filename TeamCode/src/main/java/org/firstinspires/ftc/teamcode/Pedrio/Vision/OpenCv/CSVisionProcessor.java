package org.firstinspires.ftc.teamcode.Pedrio.Vision.OpenCv;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


public class CSVisionProcessor implements VisionProcessor{
    //uncomment and determine the correct starting rectangles for your robot (If Using Java or this class directly)
    private Rect rectMiddle;// = new Rect(200, 202, 80, 80);
    private Rect rectRight;// = new Rect(300, 202, 80, 80);


    StartingPosition selection = StartingPosition.NONE;

    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    private static CSVisionProcessor _csVision;


    public static VisionProcessor getCSVision(
            int width,
            int leftX, int leftY,
            int middleX, int middleY,
            int rightX, int rightY
    ){
        _csVision = new CSVisionProcessor(width, leftX, leftY, middleX, middleY, rightX, rightY);
        return _csVision;
    }


    public static StartingPosition getPosition(){
        return _csVision.getStartingPosition();
    }


    public static int getIntPosition(){
        StartingPosition pos = _csVision.getStartingPosition();

        if(pos == StartingPosition.LEFT){
            return 1;
        }
        else if(pos == StartingPosition.CENTER){
            return 2;
        }
        else if(pos == StartingPosition.RIGHT){
            return 3;
        }

        return 0;
    }

    public CSVisionProcessor(int width, int leftX, int leftY, int middleX, int middleY, int rightX, int rightY){
        //TODO add left stuff
        rectMiddle = new Rect(middleX, middleY, width, width);
        rectRight = new Rect(rightX, rightY, width, width);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        double threshold = 32.5;

        double satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);
        double satRectRight = getAvgSaturation(hsvMat, rectRight);

        if (Math.abs(satRectMiddle - satRectRight) <= threshold) {
            selection = StartingPosition.LEFT;
        } else if (satRectMiddle > satRectRight) {
            selection = StartingPosition.CENTER;
        } else {
            selection = StartingPosition.RIGHT;
        }
        return selection;
    }


    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float  scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);

        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);

        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelected = new Paint();
        nonSelected.setStrokeWidth(scaleCanvasDensity * 4);
        nonSelected.setStyle(Paint.Style.STROKE);
        nonSelected.setColor(Color.GREEN);

        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);

        selection = (StartingPosition) userContext;

        switch(selection){
            case LEFT:
                canvas.drawRect(drawRectangleMiddle, nonSelected);
                canvas.drawRect(drawRectangleRight, nonSelected);
                break;

            case RIGHT:
                canvas.drawRect(drawRectangleMiddle, nonSelected);
                canvas.drawRect(drawRectangleRight, selectedPaint);
                break;
            case CENTER:
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelected);
                break;
            case NONE:
                canvas.drawRect(drawRectangleMiddle, nonSelected);
                canvas.drawRect(drawRectangleRight, nonSelected);
                break;

        }

    }

    public StartingPosition getStartingPosition(){
        return selection;
    }

    public enum StartingPosition{
        NONE,
        LEFT,
        RIGHT,
        CENTER
    }

}