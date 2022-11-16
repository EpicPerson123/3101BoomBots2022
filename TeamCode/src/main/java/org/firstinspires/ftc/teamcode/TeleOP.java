package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "TEST")

public class TeleOP extends LinearOpMode {

    Hardware hw = Hardware.getInstance();

    boolean goToLow = false;
    boolean goToMedium = false;
    boolean goToHigh = false;

    double lowJunction = 470;
    double medJunction = 640;
    double highJunction = 1100;

    boolean hasPressedBOnce = false;

    @Override
    public void runOpMode() throws InterruptedException {
        hw.init(hardwareMap);
        waitForStart();

        while(opModeIsActive()){

            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = (0.8 * r * Math.cos(robotAngle) - rightX * 0.8) * (1 - gamepad1.right_trigger / 1.25);
            final double v2 = (0.8 * r * Math.sin(robotAngle) + rightX * 0.8) * (1 - gamepad1.right_trigger / 1.25);
            final double v3 = (0.8 * r * Math.sin(robotAngle) - rightX * 0.8) * (1 - gamepad1.right_trigger / 1.25);
            final double v4 = (0.8 * r * Math.cos(robotAngle) + rightX * 0.8) * (1 - gamepad1.right_trigger / 1.25);


            setPower(v1,v2,v3,v4);

            telemetry.addData("ODOParallel", hw.bl.getCurrentPosition());
            telemetry.addData("ODOHorizontal", hw.fr.getCurrentPosition());
            telemetry.addData("Potentimenter", hw.armPotentiometer.getVoltage());

            if(gamepad2.x){
                hw.armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hw.armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            Orientation angles = hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Angle ", angles.firstAngle);
            telemetry.update();

            if(gamepad2.a) goToLow = !goToLow;
            /*if(gamepad2.b){
                if(hasPressedBOnce){
                    goToMedium = !goToMedium;
                }else{
                    hasPressedBOnce = true;
                }
            }*/
            if(gamepad2.y) goToHigh = !goToHigh;

            if(goToLow){
                if(hw.armMotor2.getCurrentPosition() < lowJunction - (lowJunction * 0.05) || hw.armMotor2.getCurrentPosition() > lowJunction + (lowJunction * 0.05)){
                    //not there
                    double power = 0.5;
                    if(hw.armMotor2.getCurrentPosition() > lowJunction) power = -power;
                    hw.armMotor2.setPower(power);
                    hw.armMotor1.setPower(power);
                }else{
                    //its there
                    goToLow = false;
                }
            }else if(goToMedium){
                if(hw.armMotor2.getCurrentPosition() < medJunction - (medJunction * 0.05) || hw.armMotor2.getCurrentPosition() > medJunction + (medJunction * 0.05)){
                    double power = 0.5;
                    if(hw.armMotor2.getCurrentPosition() > medJunction) power = -power;
                    hw.armMotor2.setPower(power);
                    hw.armMotor1.setPower(power);
                }else{
                    goToMedium = false;
                }
            }else if(goToHigh){
                if(hw.armMotor2.getCurrentPosition() < highJunction - (highJunction * 0.05) || hw.armMotor2.getCurrentPosition() > highJunction + (highJunction * 0.05)){
                    double power = 0.5;
                    if(hw.armMotor2.getCurrentPosition() > highJunction) power = -power;
                    hw.armMotor2.setPower(power);
                    hw.armMotor1.setPower(power);
                }else{
                    goToHigh = false;
                }
            }

            if(hw.armMotor2.getCurrentPosition() > 1500){
                hw.armMotor2.setPower(0);
                goToLow = false;
                goToMedium = false;
                goToHigh = false;
            }

            if(!goToLow && !goToMedium && !goToHigh){
                double armPower = -gamepad2.left_stick_y * 0.4;
                hw.armMotor1.setPower(armPower);
                hw.armMotor2.setPower(armPower);
            }

            double intake = gamepad2.right_trigger;
            double outtake = gamepad2.left_trigger * 0.4;
            if(intake > 0){
                hw.armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hw.armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            //double intakePower = (intake != 0) ? -intake : outtake;

            hw.armIntake.setPower(outtake - intake);
//steering reverse





            /*setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad2.left_stick_x, gamepad2.left_stick_y);
            telemetry.addData("BR motor has encoder", hw.br.getCurrentPosition());
            telemetry.addData("BL motor has encoder", hw.bl.getCurrentPosition());
            telemetry.addData("FR motor has encoder", hw.fr.getCurrentPosition());
            telemetry.addData("FL motor has encoder", hw.fl.getCurrentPosition());
            telemetry.update();
            try{
                Thread.sleep(1000);
            }catch(IllegalArgumentException ignored){

            }*/

        }




    }

    public void setPower(double fl,double fr,double bl,double br){
        hw.fl.setPower(fl);
        hw.fr.setPower(fr);
        hw.br.setPower(br);
        hw.bl.setPower(bl);

    }


}
