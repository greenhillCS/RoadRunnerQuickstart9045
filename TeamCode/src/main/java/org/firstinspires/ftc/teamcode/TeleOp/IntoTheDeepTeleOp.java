package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auton.Position.PositionStorage;

import org.firstinspires.ftc.teamcode.Config.IntoTheDeepIntakeSystem;
import org.firstinspires.ftc.teamcode.Config.IntoTheDeepSlides;

import org.firstinspires.ftc.teamcode.drive.Constants.Config.DriveConstants;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group="intothedeep", name="IntoTheDeepTeleOp")
public class IntoTheDeepTeleOp extends LinearOpMode {
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    private static final double ACCELERATION = 0.2;
    private static final double MAX_SPEED = 1.0; // Adjust this value for your desired speed

    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;

    public double x = 0,y = 0, h = 0;

    private ElapsedTime time = new ElapsedTime();

    IntoTheDeepSlides scorer;
    IntoTheDeepSlides hangar;
    IntoTheDeepIntakeSystem intake;

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
        DcMotor scoringMotor = hardwareMap.get(DcMotor.class, "SM");
        DcMotor hangingMotor = hardwareMap.get(DcMotor.class, "HM");
        DcMotor jointMotor = hardwareMap.get(DcMotor.class, "JM");
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "IM");

        //Servo Motor init
        Servo clawServo = hardwareMap.get(Servo.class, "CS");
//        CRServo intakeServo = hardwareMap.get(CRServo.class, "IS");
        CRServo intakeServo = hardwareMap.get(CRServo.class, "IS");
        RevTouchSensor scoringTouchSensor = hardwareMap.get(RevTouchSensor.class, "TS");

        scorer = new IntoTheDeepSlides(scoringMotor, telemetry, scoringTouchSensor);
//        hangar = new IntoTheDeepSlides(hangingMotor, telemetry, null);
        intake = new IntoTheDeepIntakeSystem(intakeMotor, jointMotor);//WHAT THE SIGMA

        waitForStart();

        //Drive Motor direction init
        while (opModeIsActive()) {

            switch (currentMode) {
                case DRIVER_CONTROL:
                    x = drive.getPoseEstimate().getX();
                    y = drive.getPoseEstimate().getY();
                    h = drive.getPoseEstimate().getHeading();

                    //WAYPOINT CONTROLS vvvvv
//                    if (gamepad1.a && !gamepad1.start) {
//                        //Ascent Zone Waypoint Logic----------------------------------------------------------
//                        telemetry.addData("Moving To:", "Ascent Zone");
//                        if (y <= -24){
//                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToSplineHeading(new Pose2d(-48, -36, Math.toRadians(0)))
//                                    .build());
//                        }
//                        if (y <= 24){
//                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToSplineHeading(new Pose2d(-48, 36, Math.toRadians(0)))
//                                    .build());
//                        }
//                        drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
//                                .splineToLinearHeading(new Pose2d(-22.5, -12, h), Math.toRadians(0))
//                                .build());
//                        //Debounce
//                        while(gamepad1.a){
//                            continue;
//                        }
//                        currentMode = Mode.AUTOMATIC_CONTROL;
//                    } else if (gamepad1.b && !gamepad1.start) {
//                        //Net Zone Waypoint Logic----------------------------------------------------------
//                        telemetry.addData("Moving To:", "Net Zone");
//                        drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
//                                .lineToSplineHeading(new Pose2d(-51.5, -53, Math.toRadians(225)))
//                                .build());
//                        //Debounce
//                        while(gamepad1.b){
//                            continue;
//                        }
//                        currentMode = Mode.AUTOMATIC_CONTROL;
//                    } else if (gamepad1.x) {
//                        //Submersible Zone Waypoint Logic----------------------------------------------------------
//                        telemetry.addData("Moving To:", "Submersible Zone");
//                        if (y >= -36 && x < 0){
//                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToSplineHeading(new Pose2d(-36, -36, Math.toRadians(90)))
//                                    .build());
//                        } else if (y >= -36 && x > 0){
//                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToSplineHeading(new Pose2d(36, -36, Math.toRadians(90)))
//                                    .build());
//                        }
//                        drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
//                                .lineToSplineHeading(new Pose2d(0, -32, Math.toRadians(90)))
//                                .build());
//                        // Debounce
//                        while(gamepad1.x){
//                            continue;
//                        }
//                        currentMode = Mode.AUTOMATIC_CONTROL;
//                    } else if (gamepad1.y) {
//                        // Observation Zone Waypoint Logic----------------------------------------------------------
//                        telemetry.addData("Moving To:", "Observation Zone");
//                        if (y >= -24 && x < 0){
//                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToSplineHeading(new Pose2d(-48, -36, Math.toRadians(225)))
//                                    .build());
//                        }
//                        drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
//                                .lineToSplineHeading(new Pose2d(52, -50, Math.toRadians(270)))
//                                .build());
//                        //Debounce
//                        while(gamepad1.y){
//                            continue;
//                        }
//                        currentMode = Mode.AUTOMATIC_CONTROL;
//                    } else if (gamepad1.dpad_up){
//                        //Resets the robot's position to the middle of the closest wall to the drivers
//                        drive.setPoseEstimate(new Pose2d(0, -72+(DriveConstants.BOT_LENGTH/2), Math.toRadians(90)));
//                    } else {
                        //Set's the motor's powers if no macros are being called
                        leftFrontDrive.setPower(leftFrontPower * MAX_SPEED);
                        rightFrontDrive.setPower(rightFrontPower * MAX_SPEED);
                        leftBackDrive.setPower(leftBackPower * MAX_SPEED);
                        rightBackDrive.setPower(rightBackPower * MAX_SPEED);
//                    }
                    //WAYPOINT CONTROLS ^^^^^

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

                    //HANGING SLIDE CONTROLS vvvvv
//                    if (gamepad1.dpad_left) {
//                        hangar.startPos();
//                    } else if (gamepad1.right_trigger > 0) {
//                        hangar.up(gamepad1.right_trigger);
//                    } else if (gamepad1.left_trigger > 0) {
//                        hangar.down(gamepad1.left_trigger);
//                    } else {
//                        hangar.stop();
//                    }
                    //HANGING SLIDE CONTROLS ^^^^^

                    //SCORING SLIDE CONTROLS vvvvv
                    if (gamepad2.b && !gamepad2.start) {
                        if (clawServo.getPosition() != 0) {
                            clawServo.setPosition(0);
                            time.reset();
                            while(opModeIsActive() && time.seconds() < 0.5){
                                continue;
                            }
                        }
                        scorer.startPos();
                    } else if (gamepad2.y) {
                        if (clawServo.getPosition() != 1) {
                            clawServo.setPosition(1);
                            time.reset();
                            while(opModeIsActive() && time.seconds() < 0.5){
                                continue;
                            }
                        }
                        scorer.hookPosUp();
                    } else if (gamepad2.right_trigger > 0) {
                        scorer.up(gamepad2.right_trigger);
                    } else if (gamepad2.left_trigger > 0) {
                        scorer.down(gamepad2.left_trigger);
                    }else {
                        scorer.stop();
                    }
                    //SCORING SLIDE CONTROLS ^^^^^

                    //CLAW CONTROLS vvvvv
                    if (gamepad2.x) {
                        clawServo.setPosition(0);
                    } else if (gamepad2.a) {
                        clawServo.setPosition(1);
                    }
                    //CLAW CONTROLS ^^^^^

                    //INTAKE CONTROLS vvvvv
                    if (gamepad2.right_bumper){
                        intakeServo.setPower(1);
                    } else if (gamepad2.left_bumper) {
                        intakeServo.setPower(-1);
                    } else{
                        intakeServo.setPower(0);
                    }
                    intake.update(gamepad2);
                    //INTAKE CONTROLS ^^^^^

                    break;
                case AUTOMATIC_CONTROL:
                    if(gamepad1.a||gamepad1.b||gamepad1.x||gamepad1.y){
                        drive.breakFollowing();
                        while (gamepad1.a||gamepad1.b||gamepad1.x||gamepad1.y){
                            continue;
                        }
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
//                    scorer.update();
                    break;
            }
                //WAYPOINT CONTROLS ^^^^^}

            drive.update();

            Pose2d pos = drive.getPoseEstimate();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("X", pos.getX());
            telemetry.addData("Y", pos.getY());
            telemetry.addData("Heading", Math.toDegrees(pos.getHeading()));
            telemetry.addData("Scorer Encoder", scoringMotor.getCurrentPosition());
            telemetry.addData("Scorer Power", scoringMotor.getPower());
            telemetry.addData("Joint Encoder", jointMotor.getCurrentPosition());
            telemetry.addData("Joint Power", jointMotor.getPower());
            telemetry.addData("Intake Encoder", intakeMotor.getCurrentPosition());
            telemetry.addData("Intake Power", intakeMotor.getPower());
            telemetry.update();
        }


    }

}


