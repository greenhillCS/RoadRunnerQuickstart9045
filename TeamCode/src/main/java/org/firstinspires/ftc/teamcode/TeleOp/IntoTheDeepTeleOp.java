package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.IntoTheDeepSlides;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


public class IntoTheDeepTeleOp extends LinearOpMode {
    private static final double ACCELERATION = 0.25;
    private static final double MAX_SPEED = 1.0; // Adjust this value for your desired speed

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
    private final ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode (){

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "SM");
        Servo clawServo = hardwareMap.get(Servo.class, "CS")
        IntoTheDeepSlides slides = new IntoTheDeepSlides(slideMotor); //WHAT THE SIGMA

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        while (opModeIsActive()) {
            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
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

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
//Machine that pokes eyes out
            if (gamepad2.b){
                slides.startPos();
            }else if (gamepad2.y) {
                slides.hookPos();
            }
            else if (gamepad2.right_trigger > 0){
                slides.up(gamepad1.right_trigger);
            } else if (gamepad2.left_trigger > 0) {
                slides.down(gamepad2.left_trigger);
            }
             else if (gamepad2.dpad_down){
                slides.hardPull();
            } else {
                slides.stop();
            }


            if (gamepad2.right_bumper) {
                clawServo.setPosition(1);
            } else if (gamepad2.left_bumper) {
                clawServo.setPosition(0);
            }



            telemetry.addData("Status", "Initialized");
            telemetry.update();
            while (!isStopRequested()) {
                if (gamepad1.x){
                    telemetry.addData("returning:", "yes");
                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(0, 0, 0))
                            .build());
                }else {
                    leftFrontDrive.setPower(leftFrontPower * MAX_SPEED);
                    rightFrontDrive.setPower(rightFrontPower * MAX_SPEED);
                    leftBackDrive.setPower(leftBackPower * MAX_SPEED);
                    rightBackDrive.setPower(rightBackPower * MAX_SPEED);
                }
                drive.update();

                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.update();
            }


            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }


    }

}


