package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group="drive", name="EncoderDisplay")
public class EncoderDisplayTeleOp extends LinearOpMode {
    private final String motorName0 = "SM";
    private final String motorName1 = "HM";
    private final String motorName2 = "JM";
    private final String motorName3 = "IM";

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

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Encoder Tick " + motorName0, Motor0.getCurrentPosition());
            telemetry.addData("Encoder Tick " + motorName1, Motor1.getCurrentPosition());
            telemetry.addData("Encoder Tick " + motorName2, Motor2.getCurrentPosition());
            telemetry.addData("Encoder Tick " + motorName3, Motor3.getCurrentPosition());
            telemetry.update();
        }
    }
}
