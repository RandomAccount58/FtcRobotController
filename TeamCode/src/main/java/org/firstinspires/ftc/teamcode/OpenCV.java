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
        MIDDLE,
        NOTFOUND
    }
    private Location location = Location.NOTFOUND;

    // regions of interest (needs changes prob)
    static final Rect LEFTREGION = new Rect(
            new Point(1, 1),
            new Point(256, 360)); //256, 360

    static final Rect MIDREGION = new Rect(
            new Point(512, 1), //552, 1
            new Point(768, 360)); //828, 360

    static final Rect RIGHTREGION = new Rect(
            new Point(1024, 1), //1104, 1
            new Point(1279, 360)); //1379, 360

    static double PICTURESHOLD = .2; //TODO might want to change

    public OpenCV (Telemetry t) { telemetry = t;}


    public Mat processFrame(Mat frame) {
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);

        // yellow scale

        Scalar low_ylw = new Scalar(10, 100, 20); //orange low hsv(17, 62%, 51%) rgb 255,136,0
        Scalar high_ylw = new Scalar(25, 255, 255); //orange high hsv(36, 100%, 100%)
        Core.inRange(mat, low_ylw, high_ylw, mat); //create a mask of anything considered "Orange"

        // region

        Mat left = mat.submat(LEFTREGION);
        Mat right = mat.submat(RIGHTREGION);
        Mat middle = mat.submat(MIDREGION);
        double leftV = Core.sumElems(left).val[0] /LEFTREGION.area() / 255;
        double rightV = Core.sumElems(right).val[0] / RIGHTREGION.area() / 255;
        double middleV = Core.sumElems(middle).val[0] / MIDREGION.area() / 255;
        left.release();
        right.release();
        middle.release();

        // return some info

        telemetry.addData("left value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("middle value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("right value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("left percent", Math.round(leftV * 100) + "%");
        telemetry.addData("middle percent", Math.round(middleV * 100) + "%");
        telemetry.addData("right percent", Math.round(rightV * 100) + "%");

        // where stone?

        boolean tseLeft = leftV > PICTURESHOLD;
        boolean tseRight = rightV > PICTURESHOLD;
        boolean tseMiddle = middleV > PICTURESHOLD;

        if (tseLeft) {
            location = Location.LEFT;
            telemetry.addData("capstone location", "left");
        } else if(tseMiddle) {
            location = Location.MIDDLE;
            telemetry.addData("capstone location", "middle");
        } else if(tseRight) {
            location = Location.RIGHT;
            telemetry.addData("capstone location", "right");
        } else{
            location = Location.NOTFOUND;
            telemetry.addData("capstone location", "not found");
        }

        telemetry.update();

        // bestow knowledge

        //converts image back to normal and put the rectangles on the screen
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Scalar tseFound = new Scalar(0, 255, 0);
        Scalar noTSE = new Scalar(255, 0, 0);
        Imgproc.rectangle(mat, LEFTREGION, location == Location.LEFT? tseFound:noTSE);
        Imgproc.rectangle(mat, RIGHTREGION, location == Location.RIGHT? tseFound:noTSE);
        Imgproc.rectangle(mat, MIDREGION, location == Location.MIDDLE? tseFound:noTSE);

        return mat;
    }
    public Location getLocation() {
        return location;
    }
}
