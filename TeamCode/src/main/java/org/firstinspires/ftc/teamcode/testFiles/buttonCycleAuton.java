package org.firstinspires.ftc.teamcode.testFiles;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auton.Position.PositionStorage;
import org.firstinspires.ftc.teamcode.Config.IntoTheDeepSlides;
import org.firstinspires.ftc.teamcode.drive.Constants.Config.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group="test", name="button Cycle Auton")
@Disabled
public class buttonCycleAuton extends LinearOpMode {
    TrajectorySequence entrance;
    TrajectorySequence cycle1;
    TrajectorySequence cycle2;
    TrajectorySequence cycle3;
    TrajectorySequence end;
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "SM");
        Servo clawServo = hardwareMap.get(Servo.class, "CS");
        RevTouchSensor scoringTouchSensor = hardwareMap.get(RevTouchSensor.class, "TS");

        IntoTheDeepSlides slides = new IntoTheDeepSlides(slideMotor, telemetry, scoringTouchSensor);
        ElapsedTime time = new ElapsedTime();

        drive.setPoseEstimate(new Pose2d(12, -72+(DriveConstants.BOT_LENGTH/2), Math.toRadians(90.00)));

        clawServo.setPosition(1);
        slides.moveTo(-10);
        while (slideMotor.isBusy() && !isStopRequested() && opModeIsActive()){
            continue;
        }

        entrance = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(() -> {clawServo.setPosition(1);})
                .addTemporalMarker(slides::hookPosUp)
                .splineTo(new Vector2d(-4, -24-(DriveConstants.BOT_LENGTH/2)), Math.toRadians(90))
                .build();

        cycle1 = drive.trajectorySequenceBuilder(entrance.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(36, -36, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(36, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(44, -15), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(48, -61), Math.toRadians(270))
                .addTemporalMarker(() -> {clawServo.setPosition(1);})
                .waitSeconds(1)
                .addTemporalMarker(slides::hookPosUp)
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(-2, -24-(DriveConstants.BOT_LENGTH/2), Math.toRadians(90)))
                .build();

        cycle2 = drive.trajectorySequenceBuilder(cycle1.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(46, -36, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(46, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(56, -15), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(48, -61), Math.toRadians(270))
                .addTemporalMarker(() -> {clawServo.setPosition(1);})
                .waitSeconds(1)
                .addTemporalMarker(slides::hookPosUp)
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(0, -24-(DriveConstants.BOT_LENGTH/2), Math.toRadians(90)))
                .build();

        cycle3 = drive.trajectorySequenceBuilder(cycle2.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(52, -36, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(52, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(62, -15), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(62, -50), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(48, -61), Math.toRadians(270))
                .addTemporalMarker(() -> {clawServo.setPosition(1);})
                .waitSeconds(1)
                .addTemporalMarker(slides::hookPosUp)
                .lineToLinearHeading(new Pose2d(2, -24-(DriveConstants.BOT_LENGTH/2), Math.toRadians(90)))
                .build();

        end = drive.trajectorySequenceBuilder(cycle1.end())
                .lineToLinearHeading(new Pose2d(48, -61, Math.toRadians(270)))
                .addTemporalMarker(() -> {clawServo.setPosition(1);})
                .waitSeconds(1)
                .addTemporalMarker(slides::hookPosUp)
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(4, -24-(DriveConstants.BOT_LENGTH/2), Math.toRadians(90)))
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectorySequenceAsync(entrance);
            telemetry.addData("trajectory","entrance");
            while(drive.isBusy() && opModeIsActive() && !isStopRequested()){
                drive.update();
            }
            slides.hookPosDown();
            time.reset();
            while(slideMotor.isBusy() && opModeIsActive() && !isStopRequested() && time.seconds() < 2){
                continue;
            }
            clawServo.setPosition(0.2);
            telemetry.addData("touch Sensor after claw",scoringTouchSensor.isPressed());
            telemetry.update();
            slides.startPos();

            drive.followTrajectorySequenceAsync(cycle1);
            telemetry.addData("trajectory","cycle1");
            telemetry.update();
            while(drive.isBusy() && opModeIsActive() && !isStopRequested()){
                drive.update();
            }
            slides.hookPosDown();
            time.reset();
            while(slideMotor.isBusy() && opModeIsActive() && !isStopRequested() && time.seconds() < 2){
                continue;
            }
            clawServo.setPosition(0.2);
            slides.startPos();


//            drive.followTrajectorySequenceAsync(cycle2);
//            telemetry.addData("trajectory","cycle2");
//            while(drive.isBusy() && opModeIsActive() && !isStopRequested()){
//                drive.update();
//            }
//            slides.hookPosDown();
//            time.reset();
//            while(slideMotor.isBusy() && opModeIsActive() && !isStopRequested() && time.seconds() < 2){
//                continue;
//            }
//            clawServo.setPosition(0);
//            slides.startPos();
//
//            drive.followTrajectorySequenceAsync(cycle3);
//            telemetry.addData("trajectory","cycle3");
//            while(drive.isBusy() && opModeIsActive() && !isStopRequested()){
//                drive.update();
//            }
//            slides.hookPosDown();
//            time.reset();
//            while(slideMotor.isBusy() && opModeIsActive() && !isStopRequested() && time.seconds() < 2){
//                continue;
//            }
//            clawServo.setPosition(0);
//            slides.startPos();

            drive.followTrajectorySequenceAsync(end);
            telemetry.addData("trajectory","end");
            while(drive.isBusy() && opModeIsActive() && !isStopRequested()){
                drive.update();
            }
            slides.hookPosDown();
            time.reset();
            while(slideMotor.isBusy() && opModeIsActive() && !isStopRequested() && time.seconds() < 2){
                continue;
            }
            clawServo.setPosition(0.2);
            slides.startPos();

            PositionStorage.pose = drive.getPoseEstimate();
            break;
        }
    }
}
