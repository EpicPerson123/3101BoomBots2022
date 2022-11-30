package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmStabilization implements Runnable{

    LinearOpMode auto;
    Hardware hw = Hardware.getInstance();
    double targetDistance;
    double power = 0;
    boolean doArmStuff = false;
    Telemetry telemetry;
    int nSample = 10;
    double[][] auxiliaryArr;


    public ArmStabilization(LinearOpMode a, Telemetry t){
        auto = a;
        targetDistance = 0;
        telemetry = t;
    }

    @Override
    public void run(){
        int cIndex = 0;
        auxiliaryArr = new double[nSample][];
        setAuxiliary();
        double[] values = new double[nSample];
        while(auto.opModeIsActive()){
            values[cIndex] = hw.armPotentiometer.getVoltage();
            double velocity = lagrangeDerivative(auxiliaryArr[cIndex], truncateArray(values, cIndex + 1),cIndex);
            velocity = sigmoid(velocity);
            double distanceAway = sigmoid(targetDistance - hw.armPotentiometer.getVoltage());
//            if(velocity > distanceAway){
//                power -= 0.5 * Math.abs((velocity - distanceAway));
//                power = Range.clip(power,-0.5,0.5);
//            }else{
//                power += 0.5 * Math.abs((velocity - distanceAway));
//                power = Range.clip(power,-0.5,0.5);
//            }
            //if distance away is positive, the arm is below target
            if(velocity < 0 && distanceAway > 0){
                power += 0.15;
            }else if(velocity > 0 && distanceAway > 0){
                power -= 0.03 * Math.abs(distanceAway);
            }else if(velocity < 0 && distanceAway < 0){
                power += 0.03 * Math.abs(distanceAway);
            }else if(velocity > 0 && distanceAway < 0){
                power -= 0.15;
            }
            power = Range.clip(power, -0.4, 0.6);
            telemetry.addData("Power", power);
            telemetry.addData("Velocity", velocity);
            telemetry.addData("Distance Away", distanceAway);
            telemetry.addData("Current Pos", hw.armPotentiometer.getVoltage());
            telemetry.update();
            if(doArmStuff) {
                hw.armMotor1.setPower(power);
                hw.armMotor1.setPower(power);
            }
            cIndex++;
            if(cIndex == nSample) cIndex = 0;
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

    public double lagrangeDerivative(double[] nodes, double[] values, double point) {
        double value = 0;
        for(int v = 0; v < values.length; v++) {
            //loop through all segments
            double totalSegment = 0;
            for(int n = 0; n < nodes.length; n++) {
                //loop through all values to be added
                double tempValue = 0;
                if(n != v) {
                    //loop through all derivative multiplications (for product rule)
                    double multiple = 1 / (nodes[v] - nodes[n]);
                    for(int nd = 0; nd < nodes.length; nd++) {
                        if(nd != v && nd != n) {
                            multiple *= (point - nodes[nd]) / (nodes[v] - nodes[nd]);
                        }
                    }
                    tempValue += multiple;
                }
                totalSegment += (tempValue);
            }
            value += (totalSegment * values[v]);
        }
        return value;
    }

    private double[] truncateArray(double[] totalArr, int maxIndex){
        double[] newArr = new double[maxIndex];
        for(int x = 0; x < maxIndex; x++){
            newArr[x] = totalArr[x];
        }
        return newArr;
    }

    private void setAuxiliary(){
        for(int x = 0; x < this.auxiliaryArr.length; x++){
            this.auxiliaryArr[x] = new double[x+1];
            for(int y = 0; y < x+1; y++){
                this.auxiliaryArr[x][y] = y+1;
            }
        }

    }

    public void resetPower(){
        power = 0;
    }

}
