package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group="drive", name="EncoderDisplay")
@Disabled
public class EncoderDisplayTeleOp extends LinearOpMode {
    private final String motorName0 = "SM";
    private final String motorName1 = "HM";
    private final String motorName2 = "JM";
    private final String motorName3 = "IM";
    private final String servoName0 = "CS";
    private final String servoName1 = "IS";
    private final String servoName2 = "AS";
    private final String servoName3 = "RS";

    @Override
    public void runOpMode(){
        DcMotor Motor0 = hardwareMap.get(DcMotor.class, motorName0);
        Motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotor Motor1 = hardwareMap.get(DcMotor.class, motorName1);
        Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotor Motor2 = hardwareMap.get(DcMotor.class, motorName2);
        Motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotor Motor3 = hardwareMap.get(DcMotor.class, motorName3);
        Motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Servo Servo0 = hardwareMap.get(Servo.class, servoName0);
        Servo Servo1 = hardwareMap.get(Servo.class, servoName1);
        Servo Servo2 = hardwareMap.get(Servo.class, servoName2);
        Servo Servo3 = hardwareMap.get(Servo.class, servoName3);


        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Encoder Tick " + motorName0, Motor0.getCurrentPosition());
            telemetry.addData("Encoder Tick " + motorName1, Motor1.getCurrentPosition());
            telemetry.addData("Encoder Tick " + motorName2, Motor2.getCurrentPosition());
            telemetry.addData("Encoder Tick " + motorName3, Motor3.getCurrentPosition());
            telemetry.addData("Servo Pos " + servoName0, Servo0.getPosition());
            telemetry.addData("Servo Pos " + servoName1, Servo1.getPosition());
            telemetry.addData("Servo Pos " + servoName2, Servo2.getPosition());
            telemetry.addData("Servo Pos " + servoName3, Servo3.getPosition());
            telemetry.update();
        }
    }
}
