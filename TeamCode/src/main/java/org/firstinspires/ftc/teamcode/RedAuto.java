package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    double targetArmPos;

    Pipeline pipeline = new Pipeline();
    AutoPipelineBetter pipeline2 = new AutoPipelineBetter(telemetry);

    ArmStabilization armStable;

    @Override
    public void runOpMode(){
        hw.init(hardwareMap);
        targetArmPos = hw.armPotentiometer.getVoltage();

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
        armStableThread.start();

        ODOHorizontal = hw.fr;
        ODOParallel = hw.bl;

//        driveForward(25, 0.3);
//        strafeRight(10, 0.3);
//        armMovement(0.7, 0.1);
//        hw.armIntake.setPower(1);
//        try{
//            Thread.sleep(1000);
//        }catch (Exception e){
//
//        }
//        hw.armIntake.setPower(0);
//        armMovement(0,0.1);
//        strafeLeft(10,0.3);
//        armMovement(0.3,0.3);
//        try{
//            Thread.sleep(10000);
//        }catch (Exception e){
//
//        }
//        armMovement(0.6,0.3);
//        try{
//            Thread.sleep(10000);
//        }catch (Exception e){
//
//        }
        driveForward(10,1);
        telemetry.addData("DONE with forward", "");
        telemetry.update();
        armMovement(0.5,0.5);
        telemetry.addData("DONE with arm", "");
        telemetry.update();
        driveBackwards(10,1);
        telemetry.addData("DONE with backwards", "");
        telemetry.update();
        strafeLeft(10,1);
        telemetry.addData("DONE with strafeLeft", "");
        telemetry.update();
        strafeRight(10,1);
        telemetry.addData("DONE with strafeRight", "");
        telemetry.update();




    }

    public void driveForward(double inches, double power){
        double radius = (19) * 0.0394;
        double circumference = 2 * Math.PI * radius;
        double ticksPerRotation = -1440;
        double ticks = (inches / circumference) * ticksPerRotation;

        double ODOPower;

        hw.setPower(-power,-power,-power,-power);
        while(ODOParallel.getCurrentPosition() > (ticks * 0.05) + ticks && ODOParallel.getCurrentPosition() < (ticks * 0.05) - ticks){
            double newPower = power * (1 - sigmoid(Math.abs((ODOParallel.getCurrentPosition() / ticks))));
            telemetry.addData("newPower ", newPower);
            telemetry.addData("ratio ", (ODOParallel.getCurrentPosition() / ticks));
            telemetry.addData("sigmoid ", (1 - sigmoid(Math.abs((ODOParallel.getCurrentPosition() / ticks)))));
            telemetry.update();
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
        double ticksPerRotation = -1440;
        double ticks = (inches / circumference) * ticksPerRotation;

        double ODOPower;

        hw.setPower(power,power,power,power);
        while(ODOParallel.getCurrentPosition() < (ticks * 0.05) + ticks && ODOParallel.getCurrentPosition() > (ticks * 0.05) - ticks){
            double newPower = power * (1 - sigmoid(Math.abs((ODOParallel.getCurrentPosition() / ticks))));
            hw.setPower(newPower,newPower,newPower,newPower);
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
        targetArmPos = targetVolts;
        armStable.resetPower();
        setStabilzationPos();
        double armPower = power;
        if(hw.armPotentiometer.getVoltage() > targetVolts){
            armPower *= -1;
        }
        //hw.armMotor1.setPower(armPower);
        //hw.armMotor2.setPower(armPower);
        while(hw.armPotentiometer.getVoltage() < targetVolts - (targetVolts * 0.01) || hw.armPotentiometer.getVoltage() > (targetVolts * 0.01) + targetVolts);
        hw.armMotor1.setPower(0);
        hw.armMotor2.setPower(0);
    }

    public void turn(double degrees, double power){
        Orientation angles = hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        hw.setPower(-power,power,-power,power);
        double trueAngle = (angles.firstAngle >= 0) ? 180 + angles.firstAngle : Math.abs(angles.firstAngle);
        while(trueAngle < degrees - (degrees * 0.01) || trueAngle > degrees + (degrees * 0.01)){
            angles = hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            trueAngle = (angles.firstAngle >= 0) ? 180 + angles.firstAngle : Math.abs(angles.firstAngle);
            telemetry.addData("Angle ", trueAngle);
            telemetry.update();
        }
        hw.setPower(0,0,0,0);
    }

    public void setStabilzationPos(){
        armStable.setArmTarget(this.targetArmPos);
    }

    public double sigmoid(double x){
        double yOffset = 1 / 1.7;
        double mainFunction = Math.pow(Math.E,((-2 * Math.abs(x)) - Math.log(2)));
        return (1 / (yOffset + mainFunction)) - 0.9;
    }
}
