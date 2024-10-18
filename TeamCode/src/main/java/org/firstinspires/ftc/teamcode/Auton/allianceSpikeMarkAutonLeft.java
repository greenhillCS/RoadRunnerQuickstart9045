package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Auton.Position.PositionStorage;
import org.firstinspires.ftc.teamcode.Config.IntoTheDeepSlides;
import org.firstinspires.ftc.teamcode.drive.Constants.Config.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group="intothedeep", name="allianceSpikeMarkAutonLeft")
public class allianceSpikeMarkAutonLeft extends LinearOpMode {
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "SM");
        Servo clawServo = hardwareMap.get(Servo.class, "CS");

        IntoTheDeepSlides slides = new IntoTheDeepSlides(slideMotor);

        drive.setPoseEstimate(new Pose2d(-12, -72+(DriveConstants.BOT_LENGTH/2), Math.toRadians(90.00)));

        clawServo.setPosition(1);
        slides.moveTo(10);

        TrajectorySequence t1 = drive.trajectorySequenceBuilder(new Pose2d(12, -72+(DriveConstants.BOT_LENGTH/2), Math.toRadians(90.00)))
                .splineTo(new Vector2d(0, -26-(DriveConstants.BOT_LENGTH/2)), Math.toRadians(90))
                .build();

        TrajectorySequence t2 = drive.trajectorySequenceBuilder(t1.end())
                .forward(2)
                .build();

        TrajectorySequence t3 = drive.trajectorySequenceBuilder(t2.end())
                .back(3)
                .lineToSplineHeading(new Pose2d(36, -36, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(36, -13 , Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(46,-10, Math.toRadians(90)))
                .lineTo(new Vector2d(47, -57))
                .lineTo(new Vector2d(47, -10))
                .lineToSplineHeading(new Pose2d(56, -10 , Math.toRadians(90)))
                .lineTo(new Vector2d(56, -57))
                .lineToSplineHeading(new Pose2d(58, -10 , Math.toRadians(90)))
                .lineTo(new Vector2d(60, -10))
                .lineTo(new Vector2d(60, -57))
                .build();

        waitForStart();

        while (opModeIsActive()) {
            drive.followTrajectorySequence(t1);
            slides.hookPosUp();
            while (slideMotor.isBusy() && slides.runTime.seconds() < slides.timeOutSecs && opModeIsActive()) {
                continue;
            }

            drive.followTrajectorySequence(t2);

            slides.hookPosDown();
            while (slideMotor.isBusy() && slides.runTime.seconds() < slides.timeOutSecs && opModeIsActive()) {
                continue;
            }
            clawServo.setPosition(0);

            drive.followTrajectorySequence(t3);
            
            slides.startPos();

            PositionStorage.pose = drive.getPoseEstimate();
            break;
        }
    }
}
