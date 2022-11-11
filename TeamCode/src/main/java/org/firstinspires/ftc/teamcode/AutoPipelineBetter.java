package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class AutoPipelineBetter extends OpenCvPipeline {
    public enum parkLocation{
        parkingSpot1,
        parkingSpot2,
        parkingSpot3,
    }
    Hardware robot = Hardware.getInstance();
    Mat mat = new Mat();
    parkLocation parkLoc;
    static final Rect left1 = new Rect(
            new Point(0,260),
            new Point(183,270));
    static final Rect left2 = new Rect(
            new Point(183,260),
            new Point(366,270));
    static final Rect left3 = new Rect(
            new Point(366,260),
            new Point(549,270));
    static final Rect center1 = new Rect(
            new Point(549,260),
            new Point(732,270));
    static final Rect right3 = new Rect(
            new Point(732,260),
            new Point(915,270));
    static final Rect right2 = new Rect(
            new Point(915,260),
            new Point(1098,270));
    static final Rect right1 = new Rect(
            new Point(1098,260),
            new Point(1280,270));
    Telemetry telemetry;
    public AutoPipelineBetter(Telemetry t){telemetry = t;}

    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        double hueAvg = Core.sumElems(input).val[0];
        double saturationAvg = Core.sumElems(input).val[1];
        double valueAvg = Core.sumElems(input).val[2];

        Scalar lowRGBOrange = new Scalar(0, 100, 100);
        Scalar highRBGOrange = new Scalar(30, 255, 255);

        Scalar lowRGBGreen = new Scalar(80, 75, 75);
        Scalar highRBGGreen = new Scalar(140, 255, 255);

        Scalar lowRGBBlue = new Scalar(180, 50, 50);
        Scalar highRGBBlue = new Scalar(270, 255, 255);

        Mat orangeChannel = new Mat();
        Core.inRange(mat,lowRGBOrange, highRBGOrange, orangeChannel);

        Mat greenChannel = new Mat(); // George is best :)
        Core.inRange(mat, lowRGBGreen, highRBGGreen, greenChannel);

        Mat blueChannel = new Mat();
        Core.inRange(mat, lowRGBBlue, highRGBBlue, blueChannel);

        double green = Core.sumElems(greenChannel).val[0] / (greenChannel.rows() * greenChannel.cols());
        double orange = Core.sumElems(orangeChannel).val[0] / (orangeChannel.rows() * orangeChannel.cols());
        double blue = Core.sumElems(blueChannel).val[0] / (blueChannel.rows() * blueChannel.cols());

        if(green > orange){
            //green
            parkLoc = parkLocation.parkingSpot2;
//            telemetry.addData("Green is greatest","");
//            telemetry.update();
        }else if(orange > blue){
            //orange
            parkLoc = parkLocation.parkingSpot3;
//            telemetry.addData("Orange is greatest","");
//            telemetry.update();
        }else{
            //blue
            parkLoc = parkLocation.parkingSpot1;
//            telemetry.addData("Blue is greatest","");
//            telemetry.update();
        }

//        Core.inRange(mat, lowHSV, highHSV, mat);
//        Mat left1Pixel = mat.submat(left1);
//        Mat left2Pixel = mat.submat(left2);
//        Mat left3Pixel = mat.submat(left3);
//        Mat centerPixel = mat.submat(center1);
//        Mat right3Pixel = mat.submat(right3);
//        Mat right2Pixel = mat.submat(right2);
//        Mat right1Pixel = mat.submat(right1);
//
//        // 0:l1, 1:l2, 2:l3, 3:c, 4:r3, 5:r2, 6:r1
//        double[] pixelPercents = {Core.sumElems(left1Pixel).val[0] / left1.area() / 255,Core.sumElems(left2Pixel).val[0] / left2.area() / 255,Core.sumElems(left3Pixel).val[0] / left3.area() / 255,Core.sumElems(centerPixel).val[0] / center1.area() / 255,Core.sumElems(right3Pixel).val[0] / right3.area() / 255,Core.sumElems(right2Pixel).val[0] / right2.area() / 255,Core.sumElems(right1Pixel).val[0] / right1.area() / 255};
//
//        if(pixelPercents[3] > 0.6){
//            blockLoc = blockLocation.center;
//        }else if(pixelPercents[2] > 0.6){
//            blockLoc = blockLocation.slightLeft;
//        }else if(pixelPercents[4] > 0.6){
//            blockLoc = blockLocation.slightRight;
//        }else if(pixelPercents[1] > 0.6){
//            blockLoc = blockLocation.majorLeft;
//        }else if(pixelPercents[5] > 0.6){
//            blockLoc = blockLocation.majorRight;
//        }else if(pixelPercents[0] > 0.6){
//            blockLoc = blockLocation.leftMost;
//        }else if(pixelPercents[6] > 0.6){
//            blockLoc = blockLocation.rightMost;
//        }else{
//            blockLoc = blockLocation.notFound;
//        }
        return mat;
    }

    public parkLocation getParkingSpot(){return parkLoc;}
}

