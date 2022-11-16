package org.firstinspires.ftc.teamcode;

import android.graphics.ColorSpace;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {

    Telemetry telemetry;
    public void addTelemetry(Telemetry tele){telemetry = tele;}

    int greatest = 0;
    public Mat processFrame(Mat input){
        //Mat relevantMat = input.submat(new Rect(0,0,1280,720));
        Mat relevantMat = new Mat();
        Imgproc.cvtColor(input, relevantMat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSVPink = new Scalar(262, 255, 0);
        Scalar highHSVPink = new Scalar(360,0,255);

        Core.inRange(relevantMat, lowHSVPink, highHSVPink, relevantMat);

        int purple = 0;
        int green = 0;
        int pink = 0;

        try{
            for(int pixelRow = 0; pixelRow < relevantMat.rows(); pixelRow++){
                for(int pixelCol = 0; pixelCol < relevantMat.cols(); pixelCol++){
                    double[] pixelColors = relevantMat.get(pixelCol, pixelRow);
                    if((pixelColors[0] > 50 && pixelColors[0] < 147) && (pixelColors[1] > 50 && pixelColors[1] < 82) && (pixelColors[2] == 168)){
                        //yellow pixel
                        purple++;
                    }else if((pixelColors[0] < 148 && pixelColors[0] > 3) && (pixelColors[1] == 252) && (pixelColors[2] < 115 && pixelColors[2] > 3)){
                        //green
                        green++;
                    }else if((pixelColors[0] < 168 && pixelColors[0] > 162) && (pixelColors[1] == 50) && (pixelColors[2] < 168 && pixelColors[2] > 117)){
                        //blue
                        pink++;
                    }
                }
            }
        }catch(NullPointerException e){

        }

        if(purple > green && purple > pink){
            //purple
            greatest = 0;
        }else if(green > pink){
            //green
            greatest = 1;
        }else{
            //pink
            greatest = 2;
        }

        if(Core.sumElems(relevantMat).val[0] > 0){
            telemetry.addData("Test","");
            telemetry.update();
        }

        Imgproc.cvtColor(relevantMat,relevantMat,Imgproc.COLOR_GRAY2RGB);

        return relevantMat;
    }

    public int getCurrentPark(){
        return greatest;
    }
}
