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
    ElapsedTime runTime = new ElapsedTime();
    int timeOutSecs = 3;

    double jointMax = 1.0;
    double slidesMax = 1.0;
    Telemetry telemetry;
    boolean isBusy;
    boolean wasControlled = false;
    boolean slidesTouching;
    boolean jointTouching;
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
        startPos();
    }
    public void update(Gamepad gamepad){
        if(gamepad.right_stick_y != 0 || gamepad.left_stick_y != 0) {
            wasControlled = true;

            if (isBusy) {
                isBusy = false;
                slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                joint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (touchSlides.isPressed() && !slidesTouching) {
                slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slides.setPower(0);
                slidesTouching = true;
            } else if (gamepad.right_stick_y < 0 || !touchSlides.isPressed()) {
                slides.setPower(-(gamepad.right_stick_y * slidesMax) - 0.00);
                slidesTouching = false;
            } else {
                slides.setPower(0);
            }

            if (touchJoint.isPressed() && !jointTouching) {
                joint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                joint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                joint.setPower(0);
                jointTouching = true;
            } else if (gamepad.left_stick_y > 0 || !touchJoint.isPressed()) {
                joint.setPower(-(gamepad.left_stick_y * jointMax) - 0.00);
                jointTouching = false;
            } else {
                joint.setPower(0);
            }
        } else if(wasControlled){
            slides.setPower(0);
            joint.setPower(0);
        }
    }
    public void moveTo(int pos, int angle){
        wasControlled = false;
        joint.setTargetPosition(angle);
        joint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        joint.setPower(1);
        slides.setTargetPosition(pos);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(1);
        isBusy = true;
    }
    public void startPos(){
        boolean isSlideDown = false;
//        telemetry.update();
        runTime.reset();
        slides.setTargetPosition(-10000);
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
        slidesTouching = true;

        runTime.reset();
        joint.setTargetPosition(20000);
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
        joint.setTargetPosition(-400);
        joint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        joint.setPower(1);
        while(joint.isBusy()){
            joint.setPower(1);
        }
        joint.setPower(0);
        joint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jointTouching = true;
    }
}
