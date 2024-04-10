package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class droneLauncher {
    public void droneIn(DcMotor droneMotor){
        droneMotor.setDirection(DcMotor.Direction.REVERSE);
        droneMotor.setPower(0.5);
    }
    public void droneOut(DcMotor droneMotor){
        droneMotor.setDirection(DcMotor.Direction.FORWARD);
        droneMotor.setPower(0.5);
    }
    public void droneStop(DcMotor droneMotor){
        droneMotor.setDirection(DcMotor.Direction.FORWARD);
        droneMotor.setPower(0);
    }
    public void load(Servo droneServo){
        droneServo.setPosition(1);
    }
    public void shoot(Servo droneServo) {
        droneServo.setPosition(0);
    }
}
