package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "TEST")

public class TeleOP extends LinearOpMode {

    Hardware hw = Hardware.getInstance();

    double startPotentiometer;


    @Override
    public void runOpMode(){
        hw.init(hardwareMap);
        waitForStart();
        startPotentiometer = hw.armPotentiometer.getVoltage();

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
            telemetry.addData("Potentiometer", hw.armPotentiometer.getVoltage());
            telemetry.addData("Servo ", hw.intakeServo.getPosition());

            Orientation angles = hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Angle ", angles.firstAngle);
            telemetry.update();

            double armPower = -gamepad2.left_stick_y * 0.65 * cosx(Math.abs(gamepad2.right_stick_y));
            if(!gamepad2.a && !gamepad2.x && !gamepad2.y){
                hw.armMotor1.setPower(armPower);
                hw.armMotor2.setPower(armPower);
            }
//            hw.armMotor1.setPower(armPower);
//            hw.armMotor2.setPower(armPower);

            if(gamepad2.left_trigger > 0){
                hw.intakeServo.setPosition(0.5);
            }else{
                hw.intakeServo.setPosition(0.08);
            }
            hw.armIntake.setPower(-gamepad2.right_trigger);

            if(gamepad2.left_bumper){
                hw.armIntake.setPower(0.25);
            }

            //low junction
            if(gamepad2.a){
                if(hw.armPotentiometer.getVoltage() + this.startPotentiometer > 0.186){
                    hw.armMotor1.setPower(0.05);
                    hw.armMotor2.setPower(0.05);
                }else{
                    hw.armMotor1.setPower(armPower);
                    hw.armMotor2.setPower(armPower);
                }
            }
            //med junction
            if(gamepad2.x){
                if(hw.armPotentiometer.getVoltage() + this.startPotentiometer > 0.457){
                    hw.armMotor1.setPower(0.05);
                    hw.armMotor2.setPower(0.05);
                }else{
                    hw.armMotor1.setPower(armPower);
                    hw.armMotor2.setPower(armPower);
                }
            }
            //high junction
            if(gamepad2.y){
                if(hw.armPotentiometer.getVoltage() + this.startPotentiometer > 0.690){
                    hw.armMotor1.setPower(0.05);
                    hw.armMotor2.setPower(0.05);
                }else{
                    hw.armMotor1.setPower(armPower);
                    hw.armMotor2.setPower(armPower);
                }
            }


        }

    }

    public double cosx(double x){
        return Math.cos(Math.acos(0) * x);
    }

    public void setPower(double fl,double fr,double bl,double br){
        hw.fl.setPower(fl);
        hw.fr.setPower(fr);
        hw.br.setPower(br);
        hw.bl.setPower(bl);

    }


}
