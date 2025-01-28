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

@Autonomous(group="intothedeep", name="ultimate claw cycle")
public class allianceUltimateJuicyAutonCycle extends LinearOpMode {
    enum STATE {
        ENTRANCE,
        PICKUP1,
        PICKUP2,
        PICKUP3,
        TRANSITION,
        SCORE1,
        SCORE2,
        SCORE3,
        SCORE4,
        PARK
    }
    STATE state = STATE.ENTRANCE;
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "IM");
        DcMotor jointMotor = hardwareMap.get(DcMotor.class, "JM");

        Servo clawServo = hardwareMap.get(Servo.class, "CS");
        Servo rotationServo = hardwareMap.get(Servo.class, "RS");
        Servo angleServo = hardwareMap.get(Servo.class, "AS");

        RevTouchSensor slidesTouchSensor = hardwareMap.get(RevTouchSensor.class, "TI");
        RevTouchSensor jointTouchSensor = hardwareMap.get(RevTouchSensor.class, "TA");

        Rev2mDistanceSensor distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "DS");

        Pose2d startPos = new Pose2d(12, -72+(DriveConstants.BOT_LENGTH/2), Math.toRadians(90.00));

        clawServo.setPosition(1);
        rotationServo.setPosition(0.7);
        angleServo.setPosition(0.5);

        IntoTheDeepIntakeSystem intake = new IntoTheDeepIntakeSystem(slideMotor, jointMotor, slidesTouchSensor, jointTouchSensor, telemetry);
        intake.startPos();

        clawServo.setPosition(1);
        rotationServo.setPosition(0.7);
        angleServo.setPosition(0.15);

        drive.setPoseEstimate(startPos);


        TrajectorySequence entrance =  drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .build();

        TrajectorySequence pickup1 =  drive.trajectorySequenceBuilder(entrance.end())
                .build();

        TrajectorySequence pickup2 =  drive.trajectorySequenceBuilder(pickup1.end())
                .build();

        TrajectorySequence pickup3 =  drive.trajectorySequenceBuilder(pickup2.end())
                .build();

        TrajectorySequence transition =  drive.trajectorySequenceBuilder(pickup3.end())
                .build();

        TrajectorySequence score1 =  drive.trajectorySequenceBuilder(transition.end())
                .build();

        TrajectorySequence score2 =  drive.trajectorySequenceBuilder(score1.end())
                .build();

        TrajectorySequence score3 =  drive.trajectorySequenceBuilder(score2.end())
                .build();

        TrajectorySequence score4 =  drive.trajectorySequenceBuilder(score3.end())
                .build();

        TrajectorySequence park =  drive.trajectorySequenceBuilder(score4.end())
                .build();

        waitForStart();
        drive.followTrajectorySequenceAsync(entrance);
        while (!isStopRequested() && opModeIsActive()){
            drive.update();
            switch(state){
                case ENTRANCE:
                    //Moving to the point of rotation
                    if(!drive.isBusy()) {
                        state = STATE.PICKUP1;
                        drive.followTrajectorySequenceAsync(pickup1);
                    }
                    break;
                case PICKUP1:
                    //Picking up the first sample and placing it in the observation zone
                    if(!drive.isBusy()) {
                        state = STATE.PICKUP2;
                        drive.followTrajectorySequenceAsync(pickup2);
                    }
                    break;
                case PICKUP2:
                    //Doing the same just for the second one
                    if(!drive.isBusy()) {
                        state = STATE.PICKUP3;
                        drive.followTrajectorySequenceAsync(pickup3);
                    }
                    break;
                case PICKUP3:
                    //Doing the same just for the second one
                    if(!drive.isBusy()) {
                        state = STATE.TRANSITION;
                        drive.followTrajectorySequenceAsync(transition);
                    }
                    break;
                case TRANSITION:
                    //Moving the robot to pick up the first specimen
                    if(distanceSensor.getDistance(DistanceUnit.INCH)<6 && !drive.isBusy()) {
                        state = STATE.SCORE1;
                        drive.followTrajectorySequenceAsync(score1);
                    }
                    break;
                case SCORE1:
                    //Scoring the first specimen and setting up to pick up the second specimen
                    if(distanceSensor.getDistance(DistanceUnit.INCH)<6 && !drive.isBusy()) {
                        state = STATE.SCORE2;
                        drive.followTrajectorySequenceAsync(score2);
                    }
                    break;
                case SCORE2:
                    //Scoring the second specimen and setting up to pick up the third specimen
                    if(distanceSensor.getDistance(DistanceUnit.INCH)<6 && !drive.isBusy()) {
                        state = STATE.SCORE3;
                        drive.followTrajectorySequenceAsync(score3);
                    }
                    break;
                case SCORE3:
                    //Scoring the third specimen and setting up to pick up the fourth specimen
                    if(distanceSensor.getDistance(DistanceUnit.INCH)<6 && !drive.isBusy()) {
                        state = STATE.SCORE4;
                        drive.followTrajectorySequenceAsync(score4);
                    }
                    break;
                case SCORE4:
                    //Scoring the fourth specimen and setting up to pick up the fifth specimen
                    if(distanceSensor.getDistance(DistanceUnit.INCH)<6 && !drive.isBusy()) {
                        state = STATE.PARK;
                        drive.followTrajectorySequenceAsync(park);
                    }
                    break;
                case PARK:
                    //Scoring the fifth specimen and parking
                    if(!drive.isBusy()) {
                        intake.moveTo(0, jointMotor.getCurrentPosition());
                        PositionStorage.pose = drive.getPoseEstimate();
                        requestOpModeStop();
                        break;
                    }
                    break;
            }
        }

    }
}
