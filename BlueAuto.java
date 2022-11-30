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
    double initialArmPos;

    double targetArmPos;

    Pipeline pipeline = new Pipeline();
    AutoPipelineBetter pipeline2 = new AutoPipelineBetter(telemetry);

    ArmStabilization armStable;

    @Override
    public void runOpMode(){
        //set + initialize values
        hw.init(hardwareMap);
        targetArmPos = hw.armPotentiometer.getVoltage();
        initialArmPos = hw.armPotentiometer.getVoltage();
        ODOHorizontal = hw.fr;
        ODOParallel = hw.bl;

        //camera initiation
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

        //general initiation stuff
        armStable = new ArmStabilization(this, telemetry);
        Thread armStableThread = new Thread(armStable);
        waitForStart();
        armStableThread.start();
        AutoPipelineBetter.parkLocation location = pipeline2.getParkingSpot();
        //turn on arm stabilization (do not manually call it other then here)
        armStable.setArmStuff(true);

        //methods
        //drive forwards x inches
        driveForward(0,0);
        //drive backwards x inches
        driveBackwards(0,0);
        //strafe right x inches
        strafeRight(0,0);
        //strafe left x inches
        strafeLeft(0,0);
        //turn to x degrees (except values near 0)
        turn(0,0);
        //move arm to x volt position (roughly speaking 0 is resting, 0.6 is max, and 0.3 is halfway
        armMovement(0,0);
        //servo control (0.6 = open, 0.08 = close)
        setServo(0.08);

        //Camera specific code
        switch(location){
            case parkingSpot1:
                //any code that should be run when the cone is BLUE should be placed here
                telemetry.addData("BLUE","");

                break;
            case parkingSpot2:
                //any code that should be run when the cone is GREEN should be placed here
                telemetry.addData("GREEN","");

                break;
            case parkingSpot3:
                //any code that should be run when the cone is RED should be placed here
                telemetry.addData("RED","");

                break;
        }

    }

    public void driveForward(double inches, double power){
        double radius = (19) * 0.0394;
        double circumference = 2 * Math.PI * radius;
        double ticksPerRotation = -1440;
        double ticks = (inches / circumference) * ticksPerRotation;

        double ODOPower;

        hw.setPower(-power,-power,-power,-power);
        while(Math.abs(ODOParallel.getCurrentPosition()) > (ticks * 0.01) + ticks && Math.abs(ODOParallel.getCurrentPosition()) < (ticks * 0.01) - ticks){
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
        while(Math.abs(ODOParallel.getCurrentPosition()) > (ticks * 0.01) + ticks && Math.abs(ODOParallel.getCurrentPosition()) < (ticks * 0.01) - ticks){
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

        while(ODOHorizontal.getCurrentPosition() < (ticks * 0.01) - ticks && ODOHorizontal.getCurrentPosition() > (ticks * 0.01) + ticks){
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

        while(ODOHorizontal.getCurrentPosition() > (ticks * 0.01) - ticks && ODOHorizontal.getCurrentPosition() < (ticks * 0.01) + ticks){
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
        armStable.setArmStuff(false);
        targetArmPos = targetVolts;
        //armStable.resetPower();
        double armPower = power;
        if(hw.armPotentiometer.getVoltage() > targetVolts){
            armPower *= -1;
        }
        hw.armMotor1.setPower(armPower);
        hw.armMotor2.setPower(armPower);
        while(hw.armPotentiometer.getVoltage() < targetVolts - (targetVolts * 0.025) || hw.armPotentiometer.getVoltage() > (targetVolts * 0.025) + targetVolts){

        }
        hw.armMotor1.setPower(0);
        hw.armMotor2.setPower(0);
        armStable.setArmTarget(targetVolts);
        armStable.setArmStuff(true);
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

    public void setServo(double value){
        hw.intakeServo.setPosition(value);
        try{
            Thread.sleep(500);
        }catch(Exception e){

        }
    }

    public double sigmoid(double x){
        double yOffset = 1 / 1.7;
        double mainFunction = Math.pow(Math.E,((-2 * Math.abs(x)) - Math.log(2)));
        return (1 / (yOffset + mainFunction)) - 0.9;

    }
}
