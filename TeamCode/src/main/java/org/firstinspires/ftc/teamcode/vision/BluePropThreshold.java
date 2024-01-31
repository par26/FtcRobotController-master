package org.firstinspires.ftc.teamcode.vision;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class BluePropThreshold implements VisionProcessor {
    Mat testMat = new Mat();
    Mat highMat = new Mat();
    Mat lowMat = new Mat();
    Mat finalMat = new Mat();
    double redThreshold = 0.5;

    Telemetry telemetry;
    String outStr = "left"; //Set a default value in case vision does not work

    public static Rect LEFT_RECTANGLE = new Rect(
            new Point(0, 0),
            new Point(0, 0)
    );

    public static Rect RIGHT_RECTANGLE = new Rect(
            new Point(0, 0),
            new Point(0, 0)
    );

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }


    public BluePropThreshold(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {



        telemetry.addData("[Channels]", frame.channels());


        //neccessary removes the 4th alpha channel
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGBA2RGB);

        telemetry.addData("[After Channels]", frame.channels());

        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);
        if(testMat.empty()) {
            return frame;
        }






        Scalar lowHSVRedLower = new Scalar(0, 100, 20);  //Beginning of Color Wheel
        Scalar lowHSVRedUpper = new Scalar(10, 255, 255);

        Scalar redHSVRedLower = new Scalar(160, 100, 20); //Wraps around Color Wheel
        Scalar highHSVRedUpper = new Scalar(180, 255, 255);

        Core.inRange(testMat, lowHSVRedLower, lowHSVRedUpper, lowMat);
        Core.inRange(testMat, redHSVRedLower, highHSVRedUpper, highMat);


        Core.bitwise_or(lowMat, highMat, finalMat);

        lowMat.release();
        highMat.release();

        double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
        double rightBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0];

        double averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
        double averagedRightBox = rightBox / RIGHT_RECTANGLE.area() / 255; //Makes value [0,1]

        if(averagedLeftBox > redThreshold){        //Must Tune Red Threshold
            outStr = "left";
        }else if(averagedRightBox> redThreshold){
            outStr = "center";
        }else{
            outStr = "right";
        }


        telemetry.addData("[Location]", outStr);
        telemetry.update();


        //return testMat;
        //testMat.release();


        finalMat.copyTo(frame); /*This line should only be added in when you want to see your custom pipeline
                                  //on the driver station stream, do not use this permanently in your code as
                                  //you use the "frame" mat for all of your pipelines, such as April Tags*/
        return finalMat;            //You do not return the original mat anymore, instead return null

    }


    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Rect rect = new Rect(20, 20, 50, 50);

        Rect leftRect = new Rect(LEFT_RECTANGLE.x, LEFT_RECTANGLE.y, LEFT_RECTANGLE.width, LEFT_RECTANGLE.height);
        Rect rightRect = new Rect(RIGHT_RECTANGLE.x, RIGHT_RECTANGLE.y, RIGHT_RECTANGLE.width, RIGHT_RECTANGLE.height);

        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.RED);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);

        canvas.drawRect(makeGraphicsRect(leftRect, scaleBmpPxToCanvasPx), rectPaint);
        canvas.drawRect(makeGraphicsRect(rightRect, scaleBmpPxToCanvasPx), rectPaint);


    }

    public String getPropPosition(){  //Returns postion of the prop in a String
        return outStr;
    }
}
