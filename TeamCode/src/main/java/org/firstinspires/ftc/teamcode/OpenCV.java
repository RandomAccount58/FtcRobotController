package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

// did i actually do it this time? we''ll see....

public class OpenCV extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        RIGHT,
        UNKNOWN
    }
    private Location location;

    // regions of interest (needs changes prob)
    static final Rect LEFTREGION = new Rect(
            new Point(60, 35),
            new Point(120, 75));

    static final Rect RIGHTREGION = new Rect(
            new Point(140, 35),
            new Point(200, 75));

    static double PCTHRESHOLD = .4; //TODO might want to change

    public OpenCV (Telemetry t) { telemetry = t;}

    public Mat processFrame(Mat frame) {
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);

        // yellow scale

        Scalar low_ylw = new Scalar(23, 50, 70);
        Scalar high_ylw = new Scalar(32, 255, 255);
        Core.inRange(mat, low_ylw, high_ylw, mat);

        // region

        Mat left = mat.submat((LEFTREGION));
        Mat right = mat.submat((RIGHTREGION));
        double leftV = Core.sumElems(left).val[0] /LEFTREGION.area() / 255;
        double rightV = Core.sumElems(right).val[0] / RIGHTREGION.area() / 255;
        left.release();
        right.release();

        // return some info

        telemetry.addData("left value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("right value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("left percent", Math.round(leftV * 100) + "%");
        telemetry.addData("right percent", Math.round(rightV * 100) + "%");

        // where stone?

        boolean capstoneLeft = leftV > PCTHRESHOLD;
        boolean capstoneRight = rightV > PCTHRESHOLD;

        if (capstoneLeft && capstoneRight) {
            location = Location.UNKNOWN;
            telemetry.addData("capstone location", "unknown");
        }
        if (capstoneLeft) {
            location = Location.RIGHT;
            telemetry.addData("capstone location", "right");
        }
        else{
            location = Location.LEFT;
            telemetry.addData("capstone location", "left");
        }
        telemetry.update();

        // bestow knowledge

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Scalar colorCapstone = new Scalar(0, 255, 0);
        Scalar colorStone = new Scalar(255, 0, 0);
        Imgproc.rectangle(mat, LEFTREGION, location == Location.LEFT? colorCapstone:colorStone);
        Imgproc.rectangle(mat, RIGHTREGION, location == Location.RIGHT? colorCapstone:colorStone);

        return mat;
    }
    public Location getLocation() {
        return location;
    }
}
