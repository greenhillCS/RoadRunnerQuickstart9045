package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class SlideController {
    private final DcMotor slideMotor;
    private final Gamepad gamepad;

    private static final double SLIDE_SPEED = 0.5; // Adjust this value for the desired slide speed

    public SlideController(DcMotor slideMotor, Gamepad gamepad) {
        this.slideMotor = slideMotor;
        this.gamepad = gamepad;
    }
    
     public void updateMacros () {
      if (gamepad.dpad_up){
         slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         slideMotor.setTargetPosition(-640);
         slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         slideMotor.setPower(1);
      }
     }

    public void update() {
        double slidePower = 0;

        if (gamepad.left_trigger > 0) {
            // Move the linear slide up at the specified speed
            slidePower = SLIDE_SPEED;
//            slidePower = gamepad.left_trigger * SLIDE_SPEED;
        } else if (gamepad.right_trigger > 0) {
            // Move the linear slide down at the specified speed
            slidePower = -SLIDE_SPEED;
//            slidePower = gamepad.right_trigger * SLIDE_SPEED;
        }


        //TODO: From Adi: try just using the gamepad.right_trigger and left_trigger as multipliers for the slidespeed.
        //TODO: Something like slidePower = SLIDE_SPEED * gamepad.left_trigger - this will give you more control
        //TODO: Of Course, if you don't need it, don't worry about it

        
        if (gamepad.dpad_left) {
            slidePower = 2 * SLIDE_SPEED;
        }

        slideMotor.setPower(slidePower);
    }
}
