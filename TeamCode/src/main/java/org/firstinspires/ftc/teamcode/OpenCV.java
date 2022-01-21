package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

// did i actually do it this time? we''ll see....

/* note for me */
// our robot this year is gonna use a camera
// to detect where the capstone is on the barcode

// over in the team code that will return a 0, 1, or 2
// depending on where the capstone is

public class OpenCV extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    // regions of interest (needs changes)
    static final Rect leftregion = new Rect(60, 35, 40, 40);
    static final Rect rightregion = new Rect(140, 35, 60, 40);

    public OpenCV (Telemetry t) { telemetry = t;}

    public Mat processFrame(Mat frame) {
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);

        // yellow scale

        Scalar low_ylw = new Scalar(23, 50, 70);
        Scalar high_ylw = new Scalar(32, 255, 255);
        Core.inRange(mat, low_ylw, high_ylw, mat);

        // region

        Mat left = mat.submat((leftregion));
        Mat right = mat.submat((rightregion));
        double leftV = Core.sumElems(left).val[0] / leftregion.area() / 255;
        double rightV = Core.sumElems(right).val[0] / rightregion.area() / 255;
        left.release();
        right.release();

        return frame;
    }
}
