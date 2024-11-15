package org.firstinspires.ftc.teamcode.Config;

import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.UtilityOctoQuadConfigMenu;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntoTheDeepSlides {
    private final double slowSpeed = 0.75;
    private final double fastSpeed = 1.0;
    private final double reallySlowSpeed = 0.75;
    public final int hookPos1 = 1600;
    public final int hookPos2 = 800;
    public final int hangPos = 0;
    public int timeOutSecs = 4;
    public ElapsedTime runTime = new ElapsedTime();
    private DcMotor slideMotor;
    private int targetPos = 0;
    private int t = 1;
    private Telemetry telemetry;
    private TouchSensor touchSensor;
    public boolean isSlideDown = false;



    public IntoTheDeepSlides(DcMotor motor, Telemetry te, TouchSensor touch){
        touchSensor = touch;
        telemetry = te;
        slideMotor = motor;
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        while (!isSlideDown) {
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            isSlideDown = touchSensor.isPressed();
            slideMotor.setPower(-0.2);
        }
    }
    public void moveToWait(int pos){
        if (slideMotor.getCurrentPosition()!=pos){
            slideMotor.setTargetPosition(pos);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(fastSpeed * -1);
            runTime.reset();
            while (slideMotor.isBusy() && runTime.seconds() < timeOutSecs){
                telemetry.addData("Enc Pos", slideMotor.getCurrentPosition());
                telemetry.addData("Target Pos", pos);
                telemetry.update();
                continue;
            }
            slideMotor.setPower(0);
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void moveTo(int pos){
        if (slideMotor.getCurrentPosition()!=pos){
            t = pos;
            slideMotor.setTargetPosition(pos);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(fastSpeed);
        }
    }

    public void up(double triggerSpeed){
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setPower(fastSpeed*triggerSpeed);
    }
    public void down(double triggerSpeed){
        if(!touchSensor.isPressed()){
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor.setPower(-fastSpeed*triggerSpeed);
        } else {
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void hardPull(){
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setPower(-fastSpeed);
    }
    public void stop(){
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setPower(0.000);
//        if (t == slideMotor.getCurrentPosition() || t == 1) {
//            t = 1;
//            telemetry.addData("Stopping", "Yes");
//            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            slideMotor.setPower(0.000);
//        }
//        else{
//            telemetry.addData("Stopping", "No");
//        }
    }

    public void startPos(){
        while (!isSlideDown) {
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            isSlideDown = touchSensor.isPressed();
            slideMotor.setPower(-0.2);
        }
    }
    public void hookPosUp(){
        moveTo(-hookPos1);
    }
    public void hookPosDown(){
        moveTo(-hookPos2);
    }
    public void hangPos(){
        moveTo(-hangPos);
    }



}
