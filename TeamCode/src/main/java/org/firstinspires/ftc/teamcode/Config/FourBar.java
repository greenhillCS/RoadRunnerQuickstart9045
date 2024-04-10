package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FourBar {

    public void startPos(DcMotor barMotor){
        barMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        barMotor.setTargetPosition(0);
        barMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        barMotor.setPower(1);
    }
    public void wallPos(DcMotor barMotor){
        barMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        barMotor.setTargetPosition(2347);
        barMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        barMotor.setPower(1);
    }
    public void fullDown(DcMotor barMotor){
        barMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        barMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        barMotor.setPower(1.0f);
    }
    public void rest(DcMotor barMotor){
        barMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        barMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        barMotor.setPower(0);
    }
    public void forward(DcMotor barMotor){
        barMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        barMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        barMotor.setPower(1);
    }
    public void back(DcMotor barMotor){
        barMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        barMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        barMotor.setPower(1);
    }
}
