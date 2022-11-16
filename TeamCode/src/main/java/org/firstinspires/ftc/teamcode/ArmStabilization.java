package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmStabilization implements Runnable{

    LinearOpMode auto;
    Hardware hw = Hardware.getInstance();
    double targetDistance;
    double power1 = 0;
    double power2 = 0;
    boolean doArmStuff = true;
    Telemetry telemetry;


    public ArmStabilization(LinearOpMode a, Telemetry t){
        auto = a;
        targetDistance = 0;
        telemetry = t;
    }

    @Override
    public void run(){
        while(auto.opModeIsActive()){
//            double distanceAway = hw.armPotentiometer.getVoltage() - targetDistance;
//            //power1 = ((-sigmoid(distanceAway) + hw.armMotor1.getPower()) * 0.1 < 0.1) ? power1 * ((-sigmoid(distanceAway) + hw.armMotor1.getPower())) : (-sigmoid(distanceAway) + hw.armMotor1.getPower());
//            //power1 = (-sigmoid(distanceAway) + hw.armMotor1.getPower());
//            //power1 = sigmoid(power1);
//            //power1 = (distanceAway < 0) ? power1 - sigmoid(distanceAway) : power1 - sigmoid(distanceAway) * 1.1;
//            power1 = (distanceAway < 0) ? sigmoid(distanceAway) :  sigmoid(distanceAway);
//            power1 *= -0.55;
//            telemetry.addData("Distance Away: ", distanceAway);
//            telemetry.addData("Power: ", power1);
//            telemetry.addData("Current Pos: ", hw.armPotentiometer.getVoltage());
//            telemetry.addData("target pos: ", targetDistance);
//            telemetry.update();
//            hw.armMotor1.setPower(power1);
//            hw.armMotor2.setPower(power1);
            if(doArmStuff){
                hw.armMotor1.setPower(0.02);
                hw.armMotor2.setPower(0.02);
            }

        }
    }

    public double sigmoid(double x){
       return (1 / (0.5 + Math.pow(Math.E, ((-0.15 * x) - Math.log(2))))) - 1;
    }

    public void setArmTarget(double newPos){
        targetDistance = newPos;
    }

    public double getArmTarget(){
        return targetDistance;
    }

    public void setArmStuff(boolean x){
        doArmStuff = x;
    }

    public void resetPower(){
        power1 = 0;
    }

}
