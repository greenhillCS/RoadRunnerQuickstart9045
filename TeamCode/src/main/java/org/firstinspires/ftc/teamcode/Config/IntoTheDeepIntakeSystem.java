package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class IntoTheDeepIntakeSystem {
    DcMotor joint;
    DcMotor slides;
    CRServo intake;

    double jointMax = 1.0;
    double slidesMax = 1.0;
    public IntoTheDeepIntakeSystem(DcMotor j, DcMotor s, CRServo i){
        joint = j;
        slides = s;
        intake = i;
    }
    public void update(Gamepad gamepad){
        intake.setPower(gamepad.left_stick_y);
        slides.setPower(gamepad.right_stick_x * slidesMax);
        joint.setPower(gamepad.right_stick_y * jointMax);
    }
}
