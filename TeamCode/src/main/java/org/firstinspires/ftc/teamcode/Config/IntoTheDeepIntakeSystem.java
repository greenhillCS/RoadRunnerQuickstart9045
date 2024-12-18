package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntoTheDeepIntakeSystem {
    DcMotor joint;
    DcMotor slides;
    RevTouchSensor touchSlides;
    RevTouchSensor touchJoint;
    ElapsedTime runTime;
    int timeOutSecs = 1;

    double jointMax = 1.0;
    double slidesMax = 1.0;
    Telemetry telemetry;
    public IntoTheDeepIntakeSystem(DcMotor s, DcMotor j, RevTouchSensor ts, RevTouchSensor tj, Telemetry tel){
        joint = j;
        joint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides = s;
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        touchSlides = ts;
        touchJoint = tj;
        telemetry = tel;
//        startPos();
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
    public void moveTo(int pos, int angle){
        joint.setTargetPosition(angle);
        joint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        joint.setPower(1);
        slides.setTargetPosition(pos);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        joint.setPower(1);
    }
    public void startPos(){
        boolean isSlideDown = false;
//        telemetry.update();
        runTime.reset();
        slides.setTargetPosition(10000);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (!isSlideDown && runTime.seconds()<timeOutSecs){
            isSlideDown = touchSlides.isPressed();
            slides.setPower(1);
            telemetry.addData("Is Slide Down", isSlideDown);
            telemetry.update();
        }
        telemetry.addData("Is Slide Down", isSlideDown);
        telemetry.addData("Is Slide Down", "Done");
        telemetry.update();
        slides.setPower(0);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runTime.reset();
        joint.setTargetPosition(10000);
        joint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (!touchJoint.isPressed() && runTime.seconds()<timeOutSecs){
            joint.setPower(1);
            telemetry.addData("Is Join Down", touchJoint.isPressed());
            telemetry.update();
        }
        telemetry.addData("Is Joint Down", touchJoint.isPressed());
        telemetry.addData("Is Joint Down", "Done");
        telemetry.update();
        joint.setPower(0);
        joint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        joint.setTargetPosition(-100);
        joint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        joint.setPower(1);
        while(joint.isBusy()){
            joint.setPower(1);
        }
        joint.setPower(0);
        joint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
