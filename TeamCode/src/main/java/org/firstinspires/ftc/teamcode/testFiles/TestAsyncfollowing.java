package org.firstinspires.ftc.teamcode.testFiles;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Config.IntoTheDeepSlides;
import org.firstinspires.ftc.teamcode.drive.Constants.Config.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "test", name = "Async Test")
@Disabled
public class TestAsyncfollowing extends LinearOpMode {
    public void runOpMode(){
        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "SM");
        RevTouchSensor scoringTouchSensor = hardwareMap.get(RevTouchSensor.class, "TS");

        IntoTheDeepSlides slides = new IntoTheDeepSlides(slideMotor, telemetry, scoringTouchSensor);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(12, -72+(DriveConstants.BOT_LENGTH/2), Math.toRadians(90.00)));

        TrajectorySequence t = drive.trajectorySequenceBuilder(new Pose2d(12, -72+(DriveConstants.BOT_LENGTH/2), Math.toRadians(90.00)))
                .splineTo(new Vector2d(0, -24-(DriveConstants.BOT_LENGTH/2)), Math.toRadians(90))
                .build();
        waitForStart();

        slides.hookPosUp();
        drive.followTrajectorySequenceAsync(t);
        drive.update();
        while(drive.isBusy()){
            drive.update();
        }
        slides.moveToWait(slides.hookPos2);
    }
}
