package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FullArm{
    Servo elbow1;
    Servo wrist1;
    Servo claw1;
    boolean button;

    public void ElbowFunction(Gamepad gamepad1){
        if (gamepad1.right_stick_y != 0) {
            elbow1.setPosition(elbow1.getPosition()+gamepad1.right_stick_y/1000);
        }
    }
    public void WristFunction(Gamepad gamepad1){
        if (gamepad1.left_stick_x != 0) {
            wrist1.setPosition(wrist1.getPosition()+gamepad1.left_stick_x/1000);
        }
    }
    public void ThumbFunction(Gamepad gamepad1){
        if (gamepad1.dpad_up) {
            claw1.setPosition(0);
        }
        if (gamepad1.dpad_down){
            claw1.setPosition(1);
        }
    }


    public void run(HardwareMap hardwareMap, Gamepad gamepad1){
       // elbow1 = hardwareMap.servo.get(GreenhillHardware.ELBOW_SERVO);
        wrist1 = hardwareMap.servo.get(GreenhillHardware.WRIST_SERVO);
        claw1  = hardwareMap.servo.get(GreenhillHardware.THUMB_SERVO);
      //  ElbowFunction(gamepad1);
        WristFunction(gamepad1);
        ThumbFunction(gamepad1);

    }
}
