package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
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
    public OpenCV (Telemetry t) { telemetry = t;}

    public Mat processFrame(Mat frame) {
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);
        Scalar low_ylw = new Scalar(23, 50, 70);
        Scalar high_ylw = new Scalar(32, 255, 255);
        Core.inRange(mat, low_ylw, high_ylw, mat);

        return frame;
    }
}
