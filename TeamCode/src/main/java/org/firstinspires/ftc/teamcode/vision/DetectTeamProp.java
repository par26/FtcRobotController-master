package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class DetectTeamProp implements VisionProcessor {

    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    Telemetry telemetry;

    Mat thresh = new Mat();

    double blueThreshold = 0.5;

    public String outStr = "";

    //backlog of frames to average out to reduce noise
    ArrayList<double[]> frameList;
    //these are public static to be tuned in dashboard
    public static double strictLowS = 255;
    public static double strictHighS = 255;

    static final Rect LEFT_RECTANGLE = new Rect(
            new Point(0, 0),
            new Point(0, 0)
    );

    static final Rect RIGHT_RECTANGLE = new Rect(
            new Point(0, 0),
            new Point(0, 0)
    );



    public DetectTeamProp(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        frameList = new ArrayList<>();
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return frame;
        }

        Scalar lowHSV = new Scalar(210, 100, 50); // Adjust saturation and value as needed
        Scalar highHSV = new Scalar(270, 255, 255); // Adjust saturation and value as needed


        Mat thresh = new Mat();

        // Get a black and white image of yellow objects
        Core.inRange(mat, lowHSV, highHSV, thresh);

        Mat masked = new Mat();
        //color the white portion of thresh in with HSV from mat
        //output into masked
        Core.bitwise_and(mat, mat, masked, thresh);
        //calculate average HSV values of the white thresh values
        Scalar average = Core.mean(masked, thresh);

        Mat scaledMask = new Mat();
        //scale the average saturation to 150
        masked.convertTo(scaledMask, -1, 200 / average.val[1], 0);


        Mat scaledThresh = new Mat();
        //you probably want to tune this
        Scalar strictLowHSV = new Scalar(50, strictLowS, 0); //strict lower bound HSV for yellow
        Scalar strictHighHSV = new Scalar(245, strictHighS, 255); //strict higher bound HSV for yellow
        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        Mat finalMask = new Mat();
        //color in scaledThresh with HSV, output into finalMask(only useful for showing result)(you can delete)
        Core.bitwise_and(mat, mat, finalMask, scaledThresh);



        double leftBox = Core.sumElems(finalMask.submat(LEFT_RECTANGLE)).val[0];
        double rightBox = Core.sumElems(finalMask.submat(RIGHT_RECTANGLE)).val[0];

        double averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
        double averagedRightBox = rightBox / RIGHT_RECTANGLE.area() / 255; //Makes value [0,1]

        if(averagedLeftBox > blueThreshold){        //Must Tune Red Threshold
            outStr = "left";
        }else if(averagedRightBox> blueThreshold){
            outStr = "center";
        }else{
            outStr = "right";
        }



        telemetry.addData("[Location]", outStr);
        telemetry.update();

        //release all the data
        frame.release();
        scaledThresh.copyTo(frame);
        scaledThresh.release();
        scaledMask.release();
        mat.release();
        masked.release();
        //edges.release();
        thresh.release();
        finalMask.release();
        //change the return to whatever mat you want
        //for example, if I want to look at the lenient thresh:
        // return thresh;
        // note that you must not do thresh.release() if you want to return thresh
        // you also need to release the frame if you return thresh(release as much as possible)
        return thresh;
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
}
