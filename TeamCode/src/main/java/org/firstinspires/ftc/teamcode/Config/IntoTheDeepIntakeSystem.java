package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class IntoTheDeepIntakeSystem {
    DcMotor joint;
    DcMotor slides;

    double jointMax = 1.0;
    double slidesMax = 1.0;
    public IntoTheDeepIntakeSystem(DcMotor s, DcMotor j){
        joint = j;
        joint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides = s;
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void update(Gamepad gamepad){
        if (gamepad.right_stick_y != 0) {
//            if (slides.getCurrentPosition() >= 1550 && gamepad.right_stick_y < 0) {
//                slides.setPower(0);
//            }else {
//                slides.setPower(-(gamepad.right_stick_y * slidesMax) - 0.00);
//            }
            slides.setPower(-(gamepad.right_stick_y * slidesMax) - 0.00);
        } else{
            slides.setPower(0);
        }

        joint.setPower(-gamepad.left_stick_y * jointMax);
    }
}
