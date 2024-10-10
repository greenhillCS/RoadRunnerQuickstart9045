package org.firstinspires.ftc.teamcode.testFiles;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(group="test", name = "Drone Test Tiger")
@Disabled
public class TigerTestTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor DroneMotor = hardwareMap.get(DcMotor.class, "DroneMotor");
        Servo DroneServo = hardwareMap.get(Servo.class, "DroneServo");
        DroneMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()){
            DroneMotor.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            if (gamepad1.dpad_up){
                DroneServo.setPosition(0);
            }
            if (gamepad1.dpad_down){
                DroneServo.setPosition(1);
            }
        }
    }
}
