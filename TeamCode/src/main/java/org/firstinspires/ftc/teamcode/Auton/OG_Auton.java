package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Auton.Position.PositionStorage;
import org.firstinspires.ftc.teamcode.Config.IntoTheDeepIntakeSystem;
import org.firstinspires.ftc.teamcode.Config.IntoTheDeepSlides;
import org.firstinspires.ftc.teamcode.drive.Constants.Config.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group="intothedeep", name="OG Auton")
public class OG_Auton extends LinearOpMode {
    int scorePos = 1000;
    int scoreAngle = -760;
    int scoreY = -31;

    Servo clawServo;
    Servo rotationServo;
    Servo angleServo;
    void clawControl(double angle, double rotation){
        angleServo.setPosition(angle);
        rotationServo.setPosition(rotation);
    }

    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "IM");
        DcMotor jointMotor = hardwareMap.get(DcMotor.class, "JM");

        clawServo = hardwareMap.get(Servo.class, "IS");
        rotationServo = hardwareMap.get(Servo.class, "RS");
        angleServo = hardwareMap.get(Servo.class, "AS");

        RevTouchSensor slidesTouchSensor = hardwareMap.get(RevTouchSensor.class, "TI");
        RevTouchSensor jointTouchSensor = hardwareMap.get(RevTouchSensor.class, "TA");

        Rev2mDistanceSensor distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "DS");

        Pose2d startPos = new Pose2d(12, -72+(DriveConstants.BOT_LENGTH/2), Math.toRadians(-90.00));

        clawServo.setPosition(0);
        clawControl(0, 0.3);


        IntoTheDeepIntakeSystem intake = new IntoTheDeepIntakeSystem(slideMotor, jointMotor, slidesTouchSensor, jointTouchSensor, telemetry);
        intake.startPos();

        drive.setPoseEstimate(startPos);


        TrajectorySequence trajectory =  drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawControl(0.7, 1);intake.moveTo(scorePos, scoreAngle);})//Move intake to score preload
                .setReversed(true)
                .waitSeconds(0.25)
                .splineTo(new Vector2d(5, scoreY), Math.toRadians(90))//Move robot to score preload
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawServo.setPosition(1);})//Open claw
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{intake.moveTo(0, scoreAngle);})//Move intake out of the way of the submersible

                .waitSeconds(0.25)

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(36, -36), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(36, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(44, -15), Math.toRadians(270))
                .back(43)

                .splineToConstantHeading(new Vector2d(46, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(56, -15), Math.toRadians(270))
                .back(43)


                .splineToConstantHeading(new Vector2d(52, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(62, -15), Math.toRadians(270))
                .back(43)
                .build();

        waitForStart();

        drive.followTrajectorySequenceAsync(trajectory);

        while (!isStopRequested() && opModeIsActive()){
            drive.update();

            if(!drive.isBusy()) {
                intake.startPos();
                clawServo.setPosition(0);
                rotationServo.setPosition(0.7);
                angleServo.setPosition(0);

                PositionStorage.pose = drive.getPoseEstimate();
                requestOpModeStop();
                break;
            }
        }

    }
}
