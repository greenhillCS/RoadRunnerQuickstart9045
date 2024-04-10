package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.Servo;

public class PixelRelease {
    public void open(Servo pixelServo){
        pixelServo.setPosition(0);
    }
    public void close(Servo pixelServo){
        pixelServo.setPosition(1);
    }
}
