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

@Autonomous (name = "BlueAuto")

public class BlueAuto extends LinearOpMode {
    Hardware hw = Hardware.getInstance();



    DcMotor ODOHorizontal;
    DcMotor ODOParallel;

    double targetArmPos = hw.armPotentiometer.getVoltage();

    Pipeline pipeline = new Pipeline();
    AutoPipelineBetter pipeline2 = new AutoPipelineBetter(telemetry);

    ArmStabilization armStable;

    @Override
    public void runOpMode(){
        hw.init(hardwareMap);

        hw.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                hw.camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
                pipeline.addTelemetry(telemetry);
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
        AutoPipelineBetter.parkLocation parkingSpot = pipeline2.getParkingSpot();

        driveForward(20);
        strafeLeft(6);
        armMovement(0.5);
        hw.armIntake.setPower(1);
        try{
            Thread.sleep(500);
        }catch(Exception e){};
        driveBackwards(1);
        strafeRight(6);

        switch(parkingSpot){
            case parkingSpot1:
                telemetry.addData("Code is at", "Parking spot 1");
                telemetry.update();
                strafeRight(16);
                //arm high, relsase
                break;
            case parkingSpot2:
                telemetry.addData("Code is at", "Parking spot 2");
                telemetry.update();
                strafeRight(6);
                break;
            case parkingSpot3:
                telemetry.addData("Code is at", "Parking spot 3");
                telemetry.update();
                strafeLeft(12);
                break;
        }




        ODOHorizontal = hw.fr;
        ODOParallel = hw.bl;

        telemetry.addData("Cone at", pipeline.getCurrentPark());
        telemetry.update();

        //strafeRight(20);
        //strafeLeft(20);
        //driveForward(23);



    }

    public void driveForward(double inches){
        double radius = (19) * 0.0394;
        double circumference = 2 * Math.PI * radius;
        double ticksPerRotation = -1440;
        double ticks = (inches / circumference) * ticksPerRotation;

        double ODOPower;

        hw.setPower(-0.3,-0.3,-0.3,-0.3);
        while(ODOParallel.getCurrentPosition() > (ticks * 0.05) + ticks && ODOParallel.getCurrentPosition() < (ticks * 0.05) - ticks){
            ODOPower = ODOHorizontal.getCurrentPosition() / (ticks - ODOParallel.getCurrentPosition());
            hw.addPower(ODOPower,-ODOPower,-ODOPower,ODOPower);
        }

        hw.setPower(0,0,0,0);
        ODOParallel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ODOParallel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void driveBackwards(double inches){
        double radius = (19) * 0.0394;
        double circumference = 2 * Math.PI * radius;
        double ticksPerRotation = -1440;
        double ticks = (inches / circumference) * ticksPerRotation;

        double ODOPower;

        hw.setPower(0.3,0.3,0.3,0.3);
        while(ODOParallel.getCurrentPosition() < (ticks * 0.05) + ticks && ODOParallel.getCurrentPosition() > (ticks * 0.05) - ticks){
            ODOPower = ODOHorizontal.getCurrentPosition() / (ticks - ODOParallel.getCurrentPosition());
            hw.addPower(ODOPower,-ODOPower,-ODOPower,ODOPower);
        }

        hw.setPower(0,0,0,0);
        ODOParallel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ODOParallel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void strafeLeft(double inches){
        double radius = (39) * 0.0394;
        double circumference = 2 * Math.PI * radius;
        double ticksPerRotation = -1440;
        double ticks = (inches / circumference) * ticksPerRotation;
        hw.setPower(0.3, -0.3, -0.3, 0.3);

        double ODOPower;

        while(ODOHorizontal.getCurrentPosition() < (ticks * 0.05) - ticks && ODOHorizontal.getCurrentPosition() > (ticks * 0.05) + ticks){
            ODOPower = ODOParallel.getPower() / (ticks - ODOHorizontal.getCurrentPosition());
            hw.setPower(ODOPower,ODOPower,ODOPower,ODOPower);
        }
        hw.setPower(0,0,0,0);
        ODOHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ODOHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void strafeRight(double inches){
        double radius = (39) * 0.0394;
        double circumference = 2 * Math.PI * radius;
        double ticksPerRotation = 1440;
        double ticks = (inches / circumference) * ticksPerRotation;
        hw.setPower(-0.3, 0.3, 0.3, -0.3);

        double ODOPower;

        while(ODOHorizontal.getCurrentPosition() > (ticks * 0.05) - ticks && ODOHorizontal.getCurrentPosition() < (ticks * 0.05) + ticks){
            ODOPower = ODOParallel.getPower() / (ticks - ODOHorizontal.getCurrentPosition());
            hw.addPower(ODOPower,ODOPower,ODOPower,ODOPower);
        }
        hw.setPower(0,0,0,0);
        ODOHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ODOHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void armMovement(double targetVolts){
        targetArmPos = targetVolts;
        setStabilzationPos();
        double armPower = 0.5;
        if(hw.armPotentiometer.getVoltage() > targetVolts){
            armPower *= -1;
        }
        hw.armMotor1.setPower(armPower);
        hw.armMotor2.setPower(armPower);
        while(hw.armPotentiometer.getVoltage() < (targetVolts * 0.01) - targetVolts || hw.armPotentiometer.getVoltage() > (targetVolts * 0.01) + targetVolts);
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
}
