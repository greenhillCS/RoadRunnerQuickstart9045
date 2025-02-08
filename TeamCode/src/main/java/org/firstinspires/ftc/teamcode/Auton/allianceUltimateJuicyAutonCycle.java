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
    int scorePos = 1000;
    int scoreAngle = -760;

    int pickupPos1 = 0;
    int pickupPos2 = 220;
    int pickupAngle = -4430;

    int dispensePos = 1200;
    int dispenseAngle = -4000;

    int wallPos = 0;
    int wallAngle = -4000;

    int scoreY = -31;
    int wallX = 48;
    double wallY = -72+(DriveConstants.BOT_LENGTH/2)+10;

    STATE state = STATE.ENTRANCE;
    boolean activateDistance = false;

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


        TrajectorySequence entrance =  drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawControl(0.7, 1);intake.moveTo(scorePos, scoreAngle);})//Move intake to score preload
                .setReversed(true)
                .waitSeconds(0.25)
                .splineTo(new Vector2d(5, scoreY), Math.toRadians(90))//Move robot to score preload
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawServo.setPosition(1);})//Open claw
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{intake.moveTo(0, scoreAngle);})//Move intake out of the way of the submersible

                .waitSeconds(0.25)

                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawControl(0.43, 0.3);intake.moveTo(pickupPos1, pickupAngle);})//Move intake to grab sample
                .lineToSplineHeading(new Pose2d(44, -42, Math.toRadians(75)))//Move robot to grab sample
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawServo.setPosition(0);})//Close claw on sample

                .waitSeconds(0.3)
                .build();

        TrajectorySequence pickup1 =  drive.trajectorySequenceBuilder(entrance.end())
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{intake.moveTo(dispensePos, dispenseAngle);})//Move intake to dispense sample
                .turn(Math.toRadians(-145))//Move robot to dispense sample
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawServo.setPosition(1);})//Open Claw

                .waitSeconds(0.25)

                .UNSTABLE_addTemporalMarkerOffset(0, ()->{intake.moveTo(pickupPos2, pickupAngle);})//Move intake to grab sample
                .turn(Math.toRadians(125))//Move robot to grab sample
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawServo.setPosition(0);})//Close Claw

                .waitSeconds(0.3)
                .build();

        TrajectorySequence pickup2 =  drive.trajectorySequenceBuilder(pickup1.end())
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{intake.moveTo(dispensePos, dispenseAngle);})//Move intake to dispense sample
                .turn(Math.toRadians(-145))//Move robot to dispense sample
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawServo.setPosition(1);})//Open Claw

                .waitSeconds(0.25)

//                .UNSTABLE_addTemporalMarkerOffset(0, ()->{intake.moveTo(pickupPos3, pickupAngle);})//Move intake to grab sample
//                .turn(Math.toRadians(105))//Move robot to grab sample
//                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawServo.setPosition(0);})//Close Claw

//                .waitSeconds(0.25)
                .build();

        TrajectorySequence pickup3 =  drive.trajectorySequenceBuilder(pickup2.end())
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{intake.moveTo(dispensePos, dispenseAngle);})//Move intake to dispense sample
                .turn(Math.toRadians(-105))//Move robot to dispense sample
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawServo.setPosition(1);})//Open Claw
                .build();

        TrajectorySequence transition =  drive.trajectorySequenceBuilder(pickup2.end())
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawControl(0.7, 0.3);intake.moveTo(wallPos, wallAngle);})//Move intake to pickup specimen
                .forward(-wallY-42)//Move robot to pickup specimen
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()->{activateDistance = true;})//Activate distance sensor
                .waitSeconds(0.5)
                .build();

        TrajectorySequence score1 =  drive.trajectorySequenceBuilder(transition.end())
                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawServo.setPosition(0);})//Close Claw

                .waitSeconds(0.25)

                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawControl(0.7, 1);intake.moveTo(scorePos, scoreAngle);})//Move intake to score specimen
                .waitSeconds(0.5)
                .lineTo(new Vector2d(0, -48))//Move to intermediary point
                .lineTo(new Vector2d(-4, scoreY))//Move robot to score specimen
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawServo.setPosition(1);intake.moveTo(wallPos, scoreAngle);})//Open Claw and move intake out of the way of the submersible

                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawControl(0.7, 0.3);intake.moveTo(wallPos, wallAngle);})//Move intake to pickup specimen
                .lineTo(new Vector2d(wallX, wallY+4))//Move robot to pickup specimen
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()->{activateDistance = true;})//Activate distance sensor

                .waitSeconds(0.5)
                .build();

        TrajectorySequence score2 =  drive.trajectorySequenceBuilder(score1.end())
                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawServo.setPosition(0);})//Close Claw

                .waitSeconds(0.25)

                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawControl(0.7, 1);intake.moveTo(scorePos, scoreAngle);})//Move intake to score specimen

                .waitSeconds(0.5)

                .lineTo(new Vector2d(0, -48))//Move to intermediary point
                .lineTo(new Vector2d(-3, scoreY))//Move robot to score specimen
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawServo.setPosition(1);intake.moveTo(wallPos, scoreAngle);})//Open Claw and move intake out of the way of the submersible

                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawControl(0.7, 0.3);intake.moveTo(wallPos, wallAngle);})//Move intake to pickup specimen
                .lineTo(new Vector2d(wallX, wallY+4))//Move robot to pickup specimen
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()->{activateDistance = true;})//Activate distance sensor

                .waitSeconds(0.5)
                .build();

        TrajectorySequence score3 =  drive.trajectorySequenceBuilder(score2.end())
                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawServo.setPosition(0);})//Close Claw

                .waitSeconds(0.25)

                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawControl(0.7, 1);intake.moveTo(scorePos, scoreAngle);})//Move intake to score specimen
                .lineTo(new Vector2d(0, -48))//Move to intermediary point
                .lineTo(new Vector2d(-2, scoreY))//Move robot to score specimen
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawServo.setPosition(1);})//Open Claw
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{intake.moveTo(wallPos, scoreAngle);})//Move intake out of the way of the submersible

                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawControl(0.7, 0.3);intake.moveTo(wallPos, wallAngle);})//Move intake to pickup specimen
                .lineTo(new Vector2d(wallX, wallY))//Move robot to pickup specimen
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()->{activateDistance = true;})//Activate distance sensor

                .waitSeconds(0.5)
                .build();

        TrajectorySequence park =  drive.trajectorySequenceBuilder(score2.end())

                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawServo.setPosition(0);})//Close Claw

                .waitSeconds(0.25)

                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawControl(0.7, 1);intake.moveTo(scorePos, scoreAngle);})//Move intake to score specimen

                .waitSeconds(0.5)

                .lineTo(new Vector2d(0, -48))//Move to intermediary point
                .lineTo(new Vector2d(-2, scoreY))//Move robot to score specimen
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{clawServo.setPosition(1);intake.moveTo(0, scoreAngle);})//Open Claw and move intake out of the way of the submersible

                .waitSeconds(0.25)

                .lineTo(new Vector2d(48, -58))//Move robot to park

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
                        state = STATE.TRANSITION;
                        drive.followTrajectorySequenceAsync(transition);
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
                    if((distanceSensor.getDistance(DistanceUnit.INCH)<16 && activateDistance) || !drive.isBusy()) {
                        drive.breakFollowing();
                        state = STATE.SCORE1;
                        activateDistance = false;
                        drive.followTrajectorySequenceAsync(score1);
                    }
                    break;
                case SCORE1:
                    //Scoring the first specimen and setting up to pick up the second specimen
                    if((distanceSensor.getDistance(DistanceUnit.INCH)<16 && activateDistance) || !drive.isBusy()) {
                        drive.breakFollowing();
                        state = STATE.SCORE2;
                        activateDistance = false;
                        drive.followTrajectorySequenceAsync(score2);
                    }
                    break;
                case SCORE2:
                    //Scoring the second specimen and setting up to pick up the third specimen
                    if((distanceSensor.getDistance(DistanceUnit.INCH)<16 && activateDistance) || !drive.isBusy()) {
                        drive.breakFollowing();
                        state = STATE.PARK;
                        activateDistance = false;
                        drive.followTrajectorySequenceAsync(park);
                    }
                    break;
                case SCORE3:
                    //Scoring the fourth specimen and setting up to pick up the fifth specimen
                    if((distanceSensor.getDistance(DistanceUnit.INCH)<14 && activateDistance) || !drive.isBusy()) {
                        drive.breakFollowing();
                        state = STATE.PARK;
                        activateDistance = false;
                        drive.followTrajectorySequenceAsync(park);
                    }
                    break;
                case PARK:
                    //Scoring the fifth specimen and parking
                    if(!drive.isBusy()) {
                        intake.startPos();
                        clawServo.setPosition(0);
                        rotationServo.setPosition(0.7);
                        angleServo.setPosition(0);

                        PositionStorage.pose = drive.getPoseEstimate();
                        requestOpModeStop();
                        break;
                    }
                    break;
            }
        }

    }
}
