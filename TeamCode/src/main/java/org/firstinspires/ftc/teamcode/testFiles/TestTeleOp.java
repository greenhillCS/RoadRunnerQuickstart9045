package org.firstinspires.ftc.teamcode.testFiles;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.SlideController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group="test", name="Rocket League OpMode")
@Disabled
public class TestTeleOp extends LinearOpMode{
    private final ElapsedTime runtime = new ElapsedTime();

    // Constants for acceleration and maximum speed
    private static final double ACCELERATION = 0.25;
    private static double MAX_SPEED = 0.75; // Adjust this value for your desired speed

    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;
    private double accelerate(double currentPower, double targetPower, double acceleration){
        if (currentPower < targetPower) {
            return Math.min(currentPower + acceleration, targetPower);
        } else if (currentPower > targetPower) {
            return Math.max(currentPower - acceleration, targetPower);
        }
        return targetPower;
    }

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "SM");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while(opModeIsActive()){
            double max;

            double axial = gamepad1.right_trigger - gamepad1.left_trigger;
            double lateral = 0;
            if(gamepad1.dpad_left){lateral = -1;}
            else if(gamepad1.dpad_right) {lateral = 1;}
            double yaw = -gamepad1.right_stick_x;

            telemetry.addData("Axial", axial);
            telemetry.addData("Lateral", lateral);
            telemetry.addData("yaw", yaw);

            // Calculate the target powers for each wheel
            double targetLeftFrontPower = axial + lateral + yaw;
            double targetRightFrontPower = axial - lateral - yaw;
            double targetLeftBackPower = axial - lateral + yaw;
            double targetRightBackPower = axial + lateral - yaw;

            // Apply acceleration to the wheel powers
            leftFrontPower = accelerate(leftFrontPower, targetLeftFrontPower, ACCELERATION);
            rightFrontPower = accelerate(rightFrontPower, targetRightFrontPower, ACCELERATION);
            leftBackPower = accelerate(leftBackPower, targetLeftBackPower, ACCELERATION);
            rightBackPower = accelerate(rightBackPower, targetRightBackPower, ACCELERATION);

            // Normalize the values so no wheel power exceeds 100%
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if(gamepad1.dpad_up){
                MAX_SPEED = 1;
            } else if (gamepad1.dpad_down) {
                MAX_SPEED = 0.5;
            } else{
                MAX_SPEED = 0.75;
            }
            telemetry.addData("Measured Max Speed", max);
            telemetry.addData("Max Speed", MAX_SPEED);

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            if (gamepad1.dpad_down){
                slideMotor.setPower(1.0);
            }else if(gamepad1.right_bumper){
                slideMotor.setPower(-0.5);
            }else if(gamepad1.left_bumper){
                slideMotor.setPower(0.5);
            }else{
                slideMotor.setPower(0);
            }

            leftFrontDrive.setPower(leftFrontPower * MAX_SPEED);
            rightFrontDrive.setPower(rightFrontPower * MAX_SPEED);
            leftBackDrive.setPower(leftBackPower * MAX_SPEED);
            rightBackDrive.setPower(rightBackPower * MAX_SPEED);
        }
    }
}
