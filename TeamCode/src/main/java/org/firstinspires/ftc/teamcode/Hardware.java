package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;


public class Hardware {

    DcMotor br,bl, fr,fl;
    DcMotor armMotor1, armMotor2, armIntake;
    AnalogInput armPotentiometer;
    BNO055IMU imu;

    OpenCvCamera camera;

    private static Hardware myInstance;

    public static Hardware getInstance() {
        if(myInstance == null) {
            myInstance = new Hardware();
        }
        return myInstance;
    }


    public void init(HardwareMap hwMap){
        try{
            fr = hwMap.get(DcMotor.class, "frODOH");
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fr.setDirection(DcMotorSimple.Direction.REVERSE);
            fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setPower(0);
        }catch(Exception e){
            fr = null;
        }

        try{
            fl = hwMap.get(DcMotor.class, "fl");
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //fl.setDirection(DcMotorSimple.Direction.REVERSE);
            fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fl.setPower(0);
        }catch(Exception e){
            fl = null;
        }

        try{
            br = hwMap.get(DcMotor.class, "br");
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            br.setDirection(DcMotorSimple.Direction.REVERSE);
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setPower(0);
        }catch(Exception e){
            br = null;
        }

        try{
            bl = hwMap.get(DcMotor.class, "blODOP");
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //bl.setDirection(DcMotorSimple.Direction.REVERSE);
            bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           //bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setPower(0);
        }catch(Exception e){
            bl = null;
        }

        try{
            armMotor1 = hwMap.get(DcMotor.class, "armLift1");
            armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //bl.setDirection(DcMotorSimple.Direction.REVERSE);
            armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor1.setPower(0);
        }catch(Exception e){
            armMotor1 = null;
        }

        try{
            armMotor2 = hwMap.get(DcMotor.class, "armLift2");
            armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
            armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor2.setPower(0);
        }catch(Exception e){
            armMotor2 = null;
        }

        try{
            armIntake = hwMap.get(DcMotor.class, "armIntake");
            armIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //bl.setDirection(DcMotorSimple.Direction.REVERSE);
            armIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armIntake.setPower(0);
        }catch(Exception e){
            armIntake = null;
        }

        try{
            int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
            WebcamName webcamName = hwMap.get(WebcamName.class, "Webcam");
            camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        }catch(Exception e){
            camera = null;
        }

        try{
            armPotentiometer = hwMap.analogInput.get("armPotentiometer");
        }catch(Exception e){
            armPotentiometer = null;
        }
        try{
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imu = hwMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }catch(Exception e){
            imu = null;
        }


    }


    public void setPower(double pFL, double pFR, double pBL, double pBR){
        fl.setPower(pFL);
        fr.setPower(pFR);
        bl.setPower(pBL);
        br.setPower(pBR);
    }

    public void addPower(double aFL, double aFR, double aBL, double aBR){
        fl.setPower(fl.getPower() + aFL);
        fr.setPower(fr.getPower() + aFR);
        bl.setPower(bl.getPower() + aBL);
        br.setPower(br.getPower() + aBR);
    }


    public void driveForward(int inches){
        int ticksPerRotation = 560;
        double radius = 2;
        double circumference = 2 * Math.PI * radius;

        int ticks = (int)(inches / circumference) * ticksPerRotation;

        while(bl.getCurrentPosition() < ticks && br.getCurrentPosition() < ticks){
            //fr.setPower(1);
            //fl.setPower(1);
            br.setPower(1);
            bl.setPower(1);
        }
        //fr.setPower(0);
        //fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

}
