package org.firstinspires.ftc.teamcode.testFiles;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.IntoTheDeepSlides;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(group="test",name="Test Slide Movement")
public class TestRunWhileSlidesMove extends LinearOpMode {
    TrajectorySequence t1;
    TrajectorySequence t2;

    public void runOpMode(){
        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "SM");
        RevTouchSensor touch = hardwareMap.get(RevTouchSensor.class, "TS");
        IntoTheDeepSlides slides = new IntoTheDeepSlides(slideMotor, telemetry, touch);
        ElapsedTime time = new ElapsedTime();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        slides.startPos();

        t1 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(48)
                .build();

        t2 = drive.trajectorySequenceBuilder(new Pose2d())
                .back(48)
                .build();

        waitForStart();
        slides.moveToWait(-slides.hookPos1);
        slides.startPos();
        drive.followTrajectorySequenceAsync(t1);

        while(opModeIsActive() && !isStopRequested()){
            drive.followTrajectorySequenceAsync(t1);
            while(drive.isBusy() && opModeIsActive() && !isStopRequested()){
                drive.update();
            }
            slides.hookPosDown();
            time.reset();
            while(slideMotor.isBusy() && opModeIsActive() && !isStopRequested() && time.seconds() < 2){
                continue;
            }

            slides.startPos();
            time.reset();
            while(slideMotor.getPower() != 0 && opModeIsActive() && !isStopRequested() && time.seconds() < 1){
                continue;
            }

            drive.followTrajectorySequenceAsync(t2);
            while(drive.isBusy() && opModeIsActive() && !isStopRequested()){
                drive.update();
            }
        }
    }
}
