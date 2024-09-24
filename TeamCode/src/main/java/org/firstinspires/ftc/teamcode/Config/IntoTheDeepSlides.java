package org.firstinspires.ftc.teamcode.Config;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntoTheDeepSlides {
    private final double slowSpeed = 0.5;
    private final double fastSpeed = 1.0;
    private final double reallySlowSpeed = 0.05;
    private final int hookPos = 100;
    private DcMotor slideMotor;

    public IntoTheDeepSlides(DcMotor motor){
        slideMotor = motor;
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void up(){
        slideMotor.setPower(slowSpeed);
    }
    public void down(){
        slideMotor.setPower(-slowSpeed);
    }

    public void startPos(){
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        if (slideMotor.getCurrentPosition()>0){
            slideMotor.setPower(reallySlowSpeed);
            while (slideMotor.getCurrentPosition()>0){
                continue;
            }
        }
    }
    public void hookPos(){
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        if (slideMotor.getCurrentPosition()>hookPos){
            slideMotor.setPower(reallySlowSpeed);
            while (slideMotor.getCurrentPosition()>hookPos){
                continue;
            }
        }
    }
}
