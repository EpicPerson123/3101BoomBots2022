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
    Point point1 = new Point(480,260);
    Point point2 = new Point(600,460);
    Rect elementLocation = new Rect(
            point1,
            point2);

    Telemetry telemetry;
    public AutoPipelineBetter(Telemetry t){telemetry = t;}

    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //input =  input.submat(elementLocation);

        double hueAvg = Core.sumElems(input).val[0];
        double saturationAvg = Core.sumElems(input).val[1];
        double valueAvg = Core.sumElems(input).val[2];

        Scalar lowRGBRed = new Scalar(0, 100, 100);
        Scalar highRBGRed = new Scalar(11, 255, 255);

        Scalar lowRGBGreen = new Scalar(35, 70, 80);
        Scalar highRBGGreen = new Scalar(88, 255, 255);

        Scalar lowRGBBlue = new Scalar(100, 100, 100);
        Scalar highRGBBlue = new Scalar(130, 255, 255);

        Mat redChannel = new Mat();
        Core.inRange(mat,lowRGBRed, highRBGRed, redChannel);
        redChannel = redChannel.submat(elementLocation);

        Mat greenChannel = new Mat(); // George is best :)
        Core.inRange(mat, lowRGBGreen, highRBGGreen, greenChannel);
        greenChannel = greenChannel.submat(elementLocation);

        Mat blueChannel = new Mat();
        Core.inRange(mat, lowRGBBlue, highRGBBlue, blueChannel);
        blueChannel = blueChannel.submat(elementLocation);

        Scalar rectangleColor = new Scalar(0,0,0);

        double green = Core.sumElems(greenChannel).val[0] / (greenChannel.rows() * greenChannel.cols());
        double red = Core.sumElems(redChannel).val[0] / (redChannel.rows() * redChannel.cols());
        double blue = Core.sumElems(blueChannel).val[0] / (blueChannel.rows() * blueChannel.cols());

        if(green > red){
            //green
            parkLoc = parkLocation.parkingSpot2;
            rectangleColor = new Scalar(0,255,0);
            telemetry.addData("GREEN","");
//            telemetry.addData("Green is greatest","");
//            telemetry.update();
        }else if(red > blue){
            //red
            parkLoc = parkLocation.parkingSpot3;
            rectangleColor = new Scalar(255,0,0);
            telemetry.addData("RED","");
//            telemetry.addData("Orange is greatest","");
//            telemetry.update();
        }else{
            //blue
            parkLoc = parkLocation.parkingSpot1;
            rectangleColor = new Scalar(0,0,255);
            telemetry.addData("BLUE","");
//            telemetry.addData("Blue is greatest","");
//            telemetry.update();
        }
        telemetry.update();

        Imgproc.rectangle(input, point1, point2, rectangleColor, 10);

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
        return input;
    }

    public parkLocation getParkingSpot(){return parkLoc;}
}

