package org.firstinspires.ftc.teamcode.Config;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntoTheDeepSlides {
    private final double slowSpeed = 0.5;
    private final double fastSpeed = 1.0;
    private final double reallySlowSpeed = 0.05;
    private final int hookPos = 100;
    private final int hangPos = 120;
    private DcMotor slideMotor;

    public IntoTheDeepSlides(DcMotor motor){
        slideMotor = motor;
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void moveTo(int pos){
        if (slideMotor.getCurrentPosition()!=pos){
            slideMotor.setPower(reallySlowSpeed);
            while (slideMotor.isBusy()){
                continue;
            }
        }
    }

    public void up(double triggerSpeed){
        slideMotor.setPower(slowSpeed * triggerSpeed);
    }
    public void down(double triggerSpeed){
        slideMotor.setPower(-slowSpeed * triggerSpeed);
    }
    public void hardPull(){
        slideMotor.setPower(-fastSpeed);
    }
    public void stop(){
        slideMotor.setPower(0);
    }

    public void startPos(){
        moveTo(0);
    }
    public void hookPos(){
        moveTo(hookPos);
    }
    public void hangPos(){
        moveTo(hangPos);
    }



}
