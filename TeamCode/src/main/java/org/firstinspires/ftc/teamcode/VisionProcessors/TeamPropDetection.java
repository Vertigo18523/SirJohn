package org.firstinspires.ftc.teamcode.VisionProcessors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class TeamPropDetection implements VisionProcessor {
    private boolean isBlue;

    private Telemetry telemetry;
    private BaseOpMode opMode;
    public double centerBluePercent;
    public double centerRedPercent;
    public double leftBluePercent;
    public  double leftRedPercent;
    public double rightBluePercent;
    public double rightRedPercent;
    public Rect leftRect = new Rect(0, 470, 200, 200);
    public Rect rightRect = new Rect(1080, 470, 200, 200);
    public Rect centerRect = new Rect(540, 420, 200, 200);

    public Mat rightBlueMat = new Mat(),
            rightRedMat = new Mat(),
            kernel = new Mat(),
            leftBlurredMat = new Mat(),
            leftBlueMat = new Mat(),
            centerBlueMat = new Mat(),
            leftRedMat = new Mat(),
            centerRedMat = new Mat(),
            centerBlurredMat = new Mat(),
            rightBlurredMat = new Mat();

    private static Scalar
            lowerRedBounds = new Scalar(175, 0, 0, 255),
            upperRedBounds = new Scalar(255, 175, 175, 255),
            lowerBlueBounds = new Scalar(50, 50, 100, 255),
            upperBlueBounds = new Scalar(100, 100, 255, 255);

    private volatile PropPosition position = PropPosition.RIGHT;

    public enum PropPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    public TeamPropDetection(BaseOpMode opMode, Telemetry telemetry) {
        this.opMode = opMode;
        this.telemetry = telemetry;
    }

    public void setIsBlue(boolean isBlue) {
        this.isBlue = isBlue;
    }
    public boolean getIsBlue(){
        return isBlue;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        processFrame(frame);
        return null;

    }
    public PropPosition getPosition(){
        return position;
    }
    public String getTelemetry() {
        return
                "Position: " + position + "\n" +
                        "centerBluePercent: " + centerBluePercent + "\n" +
                        "centerRedPercent: " + centerRedPercent + "\n" +
                        "rightBluePercent: " + rightBluePercent + "\n" +
                        "rightRedPercent: " + rightRedPercent + "\n" +
                        "leftBluePercent: " + leftBluePercent + "\n" +
                        "leftRedPercent: " + leftRedPercent + "\n" +
                        "isBlue: ?" + isBlue;
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
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.BLACK);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);
        Paint selectedPaint2 = new Paint();
        selectedPaint2.setColor(Color.BLACK);
        selectedPaint2.setStyle(Paint.Style.STROKE);
        selectedPaint2.setStrokeWidth(scaleCanvasDensity * 4);
        Paint selectedPaint3 = new Paint();
        selectedPaint3.setColor(Color.BLACK);
        selectedPaint3.setStyle(Paint.Style.STROKE);
        selectedPaint3.setStrokeWidth(scaleCanvasDensity * 4);

        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(leftRect, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(centerRect,scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rightRect, scaleBmpPxToCanvasPx);
        canvas.drawRect(drawRectangleLeft, selectedPaint);
        canvas.drawRect(drawRectangleMiddle, selectedPaint2);
        canvas.drawRect(drawRectangleRight, selectedPaint3);

    }


    public Mat processFrame(Mat input) {
//            telemetry.addLine("processFrame numero uno is working in a loop");

        if (opMode.getGameState() == BaseOpMode.GameState.IN_INIT) {
            telemetry.clearAll();

            Imgproc.blur(input, leftBlurredMat, new Size(5, 5));
            Imgproc.blur(input, centerBlurredMat, new Size(5, 5));
            Imgproc.blur(input, rightBlurredMat, new Size(5, 5));

            Mat tempLeft = leftBlurredMat;
            Mat tempRight = rightBlurredMat;
            Mat tempCenter = centerBlurredMat;

            leftBlurredMat = tempLeft.submat(leftRect);
            rightBlurredMat = tempRight.submat(rightRect);
            centerBlurredMat = tempCenter.submat(centerRect);

            tempLeft.release();
            tempCenter.release();
            tempRight.release();


            Core.inRange(rightBlurredMat, lowerBlueBounds, upperBlueBounds, rightBlueMat);
            Core.inRange(leftBlurredMat, lowerBlueBounds, upperBlueBounds, leftBlueMat);
            Core.inRange(centerBlurredMat, lowerBlueBounds, upperBlueBounds, centerBlueMat);


            Core.inRange(rightBlurredMat, lowerRedBounds, upperRedBounds, rightRedMat);
            Core.inRange(leftBlurredMat, lowerRedBounds, upperRedBounds, leftRedMat);
            Core.inRange(centerBlurredMat, lowerRedBounds, upperRedBounds, centerRedMat);

            centerBluePercent = Core.countNonZero(centerBlueMat);
            centerRedPercent = Core.countNonZero(centerRedMat);
            leftBluePercent = Core.countNonZero(leftBlueMat);
            leftRedPercent = Core.countNonZero(leftRedMat);
            rightBluePercent = Core.countNonZero(rightBlueMat);
            rightRedPercent = Core.countNonZero(rightRedMat);


            if (isBlue) {
                double maxBluePercent = Math.max(centerBluePercent, Math.max(leftBluePercent, rightBluePercent));
                if (maxBluePercent == centerBluePercent) {
                    position = PropPosition.CENTER;
                } else if (maxBluePercent == leftBluePercent) {
                    position = PropPosition.LEFT;
                } else if (maxBluePercent == rightBluePercent) {
                    position = PropPosition.RIGHT;
                } else {
                    telemetry.addLine("does not see anything, but knows it is blue");
                }
            } else {
                double maxRedPercent = Math.max(centerRedPercent, Math.max(leftRedPercent, rightRedPercent));
                if (maxRedPercent == centerRedPercent) {
                    position = PropPosition.CENTER;
                } else if (maxRedPercent == leftRedPercent) {
                    position = PropPosition.LEFT;
                } else if (maxRedPercent == rightRedPercent) {
                    position = PropPosition.RIGHT;
                } else {
                    telemetry.addLine("does not see anything, but knows it is red");
                }
            }

            telemetry.addData("ParkingPosition", position);

            leftBlueMat.release();
            rightBlueMat.release();
            centerBlueMat.release();
            leftRedMat.release();
            rightRedMat.release();
            centerRedMat.release();
            leftBlurredMat.release();
            centerBlurredMat.release();
            rightBlurredMat.release();

            telemetry.update();


        }
        return input;
    }

}