package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous (name = "RedAuto")

public class RedAuto extends LinearOpMode {
    Hardware hw = Hardware.getInstance();



    DcMotor ODOHorizontal;
    DcMotor ODOParallel;
    double initialArmPos;

    double targetArmPos;

    Pipeline pipeline = new Pipeline();
    AutoPipelineBetter pipeline2 = new AutoPipelineBetter(telemetry);

    ArmStabilization armStable;

    @Override
    public void runOpMode(){
        hw.init(hardwareMap);
        targetArmPos = hw.armPotentiometer.getVoltage();
        initialArmPos = hw.armPotentiometer.getVoltage();

        hw.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                hw.camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
                hw.camera.setPipeline(pipeline2);


            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Camera has Broken", "");
                telemetry.update();
            }
        });

        armStable = new ArmStabilization(this, telemetry);
        Thread armStableThread = new Thread(armStable);



        waitForStart();
        AutoPipelineBetter.parkLocation location = pipeline2.getParkingSpot();

        armStableThread.start();

        ODOHorizontal = hw.fr;
        ODOParallel = hw.bl;

        //armStable.setArmTarget(0.71);
        driveForward(61,0.8);
        armMovement(0.64, 0.70);
//        while(hw.armPotentiometer.getVoltage() > armStable.getArmTarget() + armStable.getArmTarget() * 0.05 || hw.armPotentiometer.getVoltage() < armStable.getArmTarget() - armStable.getArmTarget() * 0.05);
        try{
            Thread.sleep(800);
        }catch(Exception e){

        }
        turn(70, 0.4);
        driveForward(7.2,0.4);
        try {
            Thread.sleep(2000);
        }catch (InterruptedException e){

        }
        setServo(0.6);
        setServo(0.08);
        driveBackwards(2,0.4);
        turn(162,0.3);
        armMovement(0,0.6);
        turn(162,0.3);
        driveForward(8,0.3);
//        strafeRight(3,0.7);
//        turn(0,0.6);
//        driveForward(5,0.5);
//        //turn(0,0.6);
//        armMovement(0.65, 0.6);
//        setServo(0.6);
//        setServo(0.08);
//        driveBackwards(1,0.6);

        switch(location){
            case parkingSpot1:
                telemetry.addData("BLUE","");
                //strafeRight(12,0.5);
                strafeLeft(21,0.6);
                break;
            case parkingSpot2:
                telemetry.addData("GREEN","");
                //strafeLeft(30,0.5);
                strafeRight(21,0.6);
                break;
            case parkingSpot3:
                telemetry.addData("RED","");
                //strafeLeft(12,0.5);
                break;
        }
        //driveForward(10,-0.5);
        //turn(180,0.6);

    }

    public void driveForward(double inches, double power){
        double radius = (19) * 0.0394;
        double circumference = 2 * Math.PI * radius;
        double ticksPerRotation = -1440;
        double ticks = (inches / circumference) * ticksPerRotation;

        double ODOPower;

        hw.setPower(-power,-power,-power,-power);
        while(Math.abs(ODOParallel.getCurrentPosition()) > (ticks * 0.05) + ticks && Math.abs(ODOParallel.getCurrentPosition()) < (ticks * 0.05) - ticks){
            double newPower = power * (1 - sigmoid(Math.abs((ODOParallel.getCurrentPosition() / ticks))));
            if((ODOParallel.getCurrentPosition() / ticks) < 0.3 && power > 0 && inches > 10) newPower = 0.1 + (ODOParallel.getCurrentPosition() / ticks);
            hw.setPower(-newPower,-newPower,-newPower,-newPower);
//            ODOPower = ODOHorizontal.getCurrentPosition() / (ticks - ODOParallel.getCurrentPosition());
//            hw.addPower(ODOPower,-ODOPower,-ODOPower,ODOPower);
        }

        hw.setPower(0,0,0,0);
        ODOParallel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ODOParallel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void driveBackwards(double inches, double power){
        double radius = (19) * 0.0394;
        double circumference = 2 * Math.PI * radius;
        double ticksPerRotation = 1440;
        double ticks = (inches / circumference) * ticksPerRotation;

        double ODOPower;

        hw.setPower(power,power,power,power);
        while(!(ODOParallel.getCurrentPosition() < ticks + ticks * 0.05 && ODOParallel.getCurrentPosition() > ticks - ticks * 0.05)){
//            ODOPower = ODOHorizontal.getCurrentPosition() / (ticks - ODOParallel.getCurrentPosition());
//            hw.addPower(ODOPower,-ODOPower,-ODOPower,ODOPower);
        }

        hw.setPower(0,0,0,0);
        ODOParallel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ODOParallel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void strafeLeft(double inches, double power){
        double radius = (39) * 0.0394;
        double circumference = 2 * Math.PI * radius;
        double ticksPerRotation = -1440;
        double ticks = (inches / circumference) * ticksPerRotation;
        hw.setPower(power,-power,-power,power);

        double ODOPower;

        while(ODOHorizontal.getCurrentPosition() < (ticks * 0.05) - ticks && ODOHorizontal.getCurrentPosition() > (ticks * 0.05) + ticks){
            double newPower = power * (1 - sigmoid(Math.abs((ODOHorizontal.getCurrentPosition() / ticks))));
            hw.setPower(newPower,-newPower,-newPower,newPower);
//            ODOPower = ODOParallel.getPower() / (ticks - ODOHorizontal.getCurrentPosition());
//            hw.addPower(ODOPower,ODOPower,ODOPower,ODOPower);
        }
        hw.setPower(0,0,0,0);
        ODOHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ODOHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void strafeRight(double inches, double power){
        double radius = (39) * 0.0394;
        double circumference = 2 * Math.PI * radius;
        double ticksPerRotation = 1440;
        double ticks = (inches / circumference) * ticksPerRotation;
        hw.setPower(-power,power,power,-power);

        double ODOPower;

        while(ODOHorizontal.getCurrentPosition() > (ticks * 0.05) - ticks && ODOHorizontal.getCurrentPosition() < (ticks * 0.05) + ticks){
            double newPower = power * (1 - sigmoid(Math.abs((ODOHorizontal.getCurrentPosition() / ticks))));
            hw.setPower(-newPower,newPower,newPower,-newPower);
//            ODOPower = ODOParallel.getPower() / (ticks - ODOHorizontal.getCurrentPosition());
//            hw.addPower(ODOPower,ODOPower,ODOPower,ODOPower);
        }
        hw.setPower(0,0,0,0);
        ODOHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ODOHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void armMovement(double targetVolts, double power){
        targetVolts += initialArmPos;
        armStable.setArmStuff(false);
        targetArmPos = targetVolts;
        armStable.resetPower();
        setStabilzationPos();
        double armPower = power;
        if(hw.armPotentiometer.getVoltage() > targetVolts){
            armPower *= -1;
        }
        hw.armMotor1.setPower(armPower);
        hw.armMotor2.setPower(armPower);
        while(hw.armPotentiometer.getVoltage() < targetVolts - (targetVolts * 0.05) || hw.armPotentiometer.getVoltage() > (targetVolts * 0.05) + targetVolts){

        }
        hw.armMotor1.setPower(0);
        hw.armMotor2.setPower(0);
        armStable.setArmStuff(true);
    }

    public void turn(double degrees, double power){
        Orientation angles = hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        hw.setPower(-power,power,-power,power);
        double trueAngle = (angles.firstAngle >= 0) ? 180 + angles.firstAngle : Math.abs(angles.firstAngle);
        //trueAngle = Math.abs(angles.firstAngle);
        while(trueAngle < degrees - (degrees * 0.05) || trueAngle > degrees + (degrees * 0.05)){
            angles = hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            trueAngle = (angles.firstAngle >= 0) ? 180 + angles.firstAngle : Math.abs(angles.firstAngle);
            //trueAngle = Math.abs(angles.firstAngle);
            telemetry.addData("Angle ", trueAngle);
            telemetry.update();
        }
        hw.setPower(0,0,0,0);
    }

    public void setServo(double value){
        hw.intakeServo.setPosition(value);
        try{
            Thread.sleep(1000);
        }catch(Exception e){

        }
    }

    public void setStabilzationPos(){
        armStable.setArmTarget(this.targetArmPos + this.initialArmPos);
    }

    public double sigmoid(double x){
        double yOffset = 1 / 1.7;
        double mainFunction = Math.pow(Math.E,((-2 * Math.abs(x)) - Math.log(2)));
        return (1 / (yOffset + mainFunction)) - 0.9;

    }
}
