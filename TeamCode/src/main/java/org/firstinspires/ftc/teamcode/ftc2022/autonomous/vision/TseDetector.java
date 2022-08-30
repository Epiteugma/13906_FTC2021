package org.firstinspires.ftc.teamcode.ftc2022.autonomous.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

public class TseDetector extends OpenCvPipeline {

    static double THRESHOLD = 0.15f;

    public enum Location {
        LEFT,
        RIGHT,
        CENTER
    }

//    Telemetry telemetry;
    Mat mat = new Mat();
    private Location location;

    static final Rect LEFT_ROI = new Rect(
            new Point(0, 100),
            new Point(106, 240)
    );
    static final Rect CENTER_ROI = new Rect(
            new Point(106, 100),
            new Point(212, 240)
    );
    static final Rect RIGHT_ROI = new Rect(
            new Point(212, 100),
            new Point(320, 240)
    );

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(0, 195, 55);
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

        double[] values = { leftValue, centerValue, rightValue };

        Arrays.sort(values);
        if(values[2] > THRESHOLD) {
            if(leftValue == values[2]) location = Location.LEFT;
            else if(rightValue == values[2]) location = Location.RIGHT;
            else if(centerValue == values[2]) location = Location.CENTER;
        }

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
