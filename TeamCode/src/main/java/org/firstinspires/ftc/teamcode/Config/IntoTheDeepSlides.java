package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntoTheDeepSlides {
    private final double slowSpeed = 0.75;
    private final double fastSpeed = 1.0;
    private final double reallySlowSpeed = 0.75;
    private final int hookPos1 = 1550;
    private final int hookPos2 = 800;
    private final int hangPos = 0;
    public int timeOutSecs = 4;
    public ElapsedTime runTime = new ElapsedTime();
    private DcMotor slideMotor;

    public IntoTheDeepSlides(DcMotor motor){
        slideMotor = motor;
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void moveTo(int pos){
        if (slideMotor.getCurrentPosition()!=pos){
            slideMotor.setTargetPosition(pos);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(fastSpeed);
            runTime.reset();
            while (slideMotor.isBusy() & runTime.seconds() < timeOutSecs){
                continue;
            }
            slideMotor.setPower(0);
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    public void hookPosUp(){
        moveTo(hookPos1);
    }
    public void hookPosDown(){
        moveTo(hookPos2);
    }
    public void hangPos(){
        moveTo(hangPos);
    }



}
