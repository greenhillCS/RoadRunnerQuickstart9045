package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group="drive", name="EncoderDisplay")
public class EncoderDisplayTeleOp extends LinearOpMode {
    private String motorName = "SM";

    @Override
    public void runOpMode(){
        DcMotor Motor = hardwareMap.get(DcMotor.class, motorName);
        Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Encoder Tick", Motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
