package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auton.Position.PositionStorage;
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

        //Mecanum Drive Class init
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(PositionStorage.pose);

        //Drive Motor init
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //DC Motor init
        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "SM");

        //Servo Motor init
        Servo clawServo = hardwareMap.get(Servo.class, "CS");

        IntoTheDeepSlides slides = new IntoTheDeepSlides(slideMotor); //WHAT THE SIGMA

        //Drive Motor direction init
        while (opModeIsActive()) {


            //DRIVE CONTROLS vvvvv
            double max;
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

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
            //DRIVE CONTROLS ^^^^^

            //SLIDE CONTROLS vvvvv
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
             //SLIDE CONTROLS ^^^^^

            //CLAW CONTROLS vvvvv
            if (gamepad2.right_bumper) {
                clawServo.setPosition(1);
            } else if (gamepad2.left_bumper) {
                clawServo.setPosition(0);
            }
            //CLAW CONTROLS ^^^^^



            telemetry.addData("Status", "Initialized");
            telemetry.update();
            //WAYPOINT CONTROLS vvvvv
            if (gamepad1.a){
                telemetry.addData("Moving To:", "Ascent Zone");
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(0, 0, 0))
                        .build());
            } else if(gamepad1.b) {
                telemetry.addData("Moving To:", "Net Zone");
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(0, 0, 0))
                        .build());
            } else if(gamepad1.x) {
                telemetry.addData("Moving To:", "Submersible Zone");
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(0, 0, 0))
                        .build());
            } else if(gamepad1.y) {
                telemetry.addData("Moving To:", "Observation Zone");
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(0, 0, 0))
                        .build());
            } else {
                leftFrontDrive.setPower(leftFrontPower * MAX_SPEED);
                rightFrontDrive.setPower(rightFrontPower * MAX_SPEED);
                leftBackDrive.setPower(leftBackPower * MAX_SPEED);
                rightBackDrive.setPower(rightBackPower * MAX_SPEED);
            }
            //WAYPOINT CONTROLS ^^^^^

            drive.update();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();



            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }


    }

}


