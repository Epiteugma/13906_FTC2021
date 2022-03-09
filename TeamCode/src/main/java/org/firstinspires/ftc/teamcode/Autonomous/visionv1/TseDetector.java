package org.firstinspires.ftc.teamcode.Autonomous.visionv1;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
// import org.firstinspires.ftc.teamcode.vision.Location;


public class TseDetector extends OpenCvPipeline {

    static double TRESHOLD = 0.6;

    public enum Location {
        LEFT,
        RIGHT,
        CENTER
    }

//    Telemetry telemetry;
    Mat mat = new Mat();
    private Location location;

    static final Rect LEFT_ROI = new Rect(
            new Point(100, 500),
            new Point(400, 800));
    static final Rect CENTER_ROI = new Rect(
            new Point(500, 500),
            new Point(800, 800));
    static final Rect RIGHT_ROI = new Rect(
            new Point(900, 500),
            new Point(1200, 800));

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        // TODO: change this to HSV values for the tse
        Scalar lowHSV = new Scalar(0, 20, 40);
        Scalar highHSV = new Scalar(10, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        Mat center = mat.submat(CENTER_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;

        left.release();
        right.release();
        center.release();
        
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

//        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
//        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
//        telemetry.addData("Center percentage", Math.round(centerValue * 100) + "%");

        boolean itemLeft = leftValue > TRESHOLD;
        boolean itemRight = rightValue > TRESHOLD;
        boolean itemCenter = centerValue > TRESHOLD;

        if (itemLeft) {
            location = Location.LEFT;
//            telemetry.addData("Location", "LEFT");
        }
        else if (itemRight) {
            location = Location.RIGHT;
//            telemetry.addData("Location", "RIGHT");
        }
        else {
            location = Location.CENTER;
//            telemetry.addData("Location", "CENTER");
        }
//        telemetry.update();

        //TODO: Check if it detects tse(change the hsv values)

        Scalar found = new Scalar(0, 255, 0);
        Scalar not_found = new Scalar(255, 0, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? found:not_found);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? found:not_found);
        Imgproc.rectangle(mat, CENTER_ROI, location == Location.CENTER? found:not_found);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}
