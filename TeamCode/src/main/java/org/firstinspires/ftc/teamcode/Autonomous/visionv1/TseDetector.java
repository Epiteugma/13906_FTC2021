package org.firstinspires.ftc.teamcode.Autonomous.visionv1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
// import org.firstinspires.ftc.teamcode.vision.Location;


public class TseDetector extends OpenCvPipeline {

    static double TRESHOLD = 0.15f;

    public enum Location {
        LEFT,
        RIGHT,
        CENTER
    }

//    Telemetry telemetry;
    Mat mat = new Mat();
    private Location location;

//    static final Rect LEFT_ROI = new Rect(
//            new Point(25, 80),
//            new Point(95, 190));
//    static final Rect CENTER_ROI = new Rect(
//            new Point(125, 80),
//            new Point(195, 190));
//    static final Rect RIGHT_ROI = new Rect(
//            new Point(230, 80),
//            new Point(300, 190));

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
        // TODO: change this to HSV values for the tse
        Scalar lowHSV = new Scalar(4, 170, 45);
        Scalar highHSV = new Scalar(30, 255, 255);

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

//        boolean itemLeft = leftValue > TRESHOLD;
//        boolean itemRight = rightValue > TRESHOLD;
//        boolean itemCenter = centerValue > TRESHOLD;

        double[] values = { leftValue, centerValue, rightValue };

        Arrays.sort(values);
        if(values[2] > TRESHOLD) {
            if(leftValue == values[2]) location = Location.LEFT;
            else if(rightValue == values[2]) location = Location.RIGHT;
            else if(centerValue == values[2]) location = Location.CENTER;
        }

        //TODO: Check if it detects tse(change the hsv values)

        Scalar found = new Scalar(0, 255, 0);
        Scalar not_found = new Scalar(255, 0, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? found:not_found);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? found:not_found);
        Imgproc.rectangle(mat, CENTER_ROI, location == Location.CENTER? found:not_found);

        return mat;
    }

    public Location getLocation(LinearOpMode opMode) {
        while(true) {
            if (location != null || opMode.isStopRequested()) {
                break;
            }
        }
        return location;
    }
}
