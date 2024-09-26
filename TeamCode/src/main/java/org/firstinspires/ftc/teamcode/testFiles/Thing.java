
package org.firstinspires.ftc.teamcode.testFiles;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Config.GreenhillHardware;

@TeleOp(group = "test", name = "thing")

public class Thing extends LinearOpMode {
 Servo pusher;
 DcMotor thrower;

 @Override
 public void runOpMode(){
  thrower = hardwareMap.get(DcMotor.class, "droneMotor");
  pusher = hardwareMap.get(Servo.class, "droneServo");
  waitForStart();
  while(opModeIsActive()) {
   if (gamepad1.a) {
    thrower.setDirection(DcMotor.Direction.REVERSE);
    thrower.setPower(0.5);

   }
   else if (gamepad1.x) {
    thrower.setDirection(DcMotor.Direction.FORWARD);
    thrower.setPower(0.5);
   }
   else if (gamepad1.b){
    thrower.setDirection(DcMotor.Direction.REVERSE);
    thrower.setPower(0);
   }
   if (gamepad1.left_bumper) {
    pusher.setPosition(0);
   }
   else if (gamepad1.right_bumper) {
    pusher.setPosition(1);
   }
  }
 }



}
